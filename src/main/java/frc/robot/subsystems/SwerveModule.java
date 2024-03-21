package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import lib.util.REVSpark;
import lib.util.Util;

public class SwerveModule extends SubsystemBase {
  // Various motors FIXME: clarification needed
  private final TalonFX driveMotor;

  // Allows full use of 15% power FIXME: clarification needed
  private final VelocityVoltage driveMotorRequest;

  // Controller for the rotation motor
  private final REVSpark turnMotor;

  // For making SmartDashboard values easily discernible
  private final String niceName;

  // The requested target angle and velocity of the swerve module
  private volatile double targetAngle;
  private volatile double targetVelocity;
  private double lastTurnValue;

  /**
    * Initializes one module for a swerve drive robot
    * @param drivePort The port ID of the drive motor's controller
    * @param turnPort The port ID of the rotation motor's controller
    * @param kencoderOffset The initial offset value of the absolute encoder
    * @param niceName Pretty name for easier debugging
   **/
  public SwerveModule(int drivePort, int turnPort, double kencoderOffset, String niceName) {
    this.niceName = niceName;

    // TalonFX doesn't use RIO canbus, it uses its own
    this.driveMotor = new TalonFX(drivePort, Constants.Ports.CTRE_CANBUS);

    // Enables FOC (15% extra power) FIXME: clarification needed
//    this.driveMotorRequest = new VoltageOut(0).withEnableFOC(true);
    this.driveMotorRequest = new VelocityVoltage(0).withEnableFOC(true);
    Slot0Configs driveMotorSlot0 = new Slot0Configs()
      .withKP(Constants.Drivetrain.DRIVE_P)
      .withKI(Constants.Drivetrain.DRIVE_I)
      .withKD(Constants.Drivetrain.DRIVE_D)
      .withKS(Constants.Drivetrain.MODULE_S)
      .withKV(Constants.Drivetrain.MODULE_V)
      .withKA(Constants.Drivetrain.MODULE_A);

    this.driveMotor.setPosition(0.0);
    CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs().withStatorCurrentLimit(Constants.Drivetrain.DRIVE_CURRENT_LIMIT).withStatorCurrentLimitEnable(true);

    // 1 / ((1 / gear ratio) * pi * diameter) Solved on whiteboard photo in drive
    FeedbackConfigs feedbackConfigs = new FeedbackConfigs().withSensorToMechanismRatio(Constants.Drivetrain.DRIVE_GEAR_RATIO / Units.inchesToMeters(Math.PI * Constants.Drivetrain.WHEEL_DIAMETER));
    TalonFXConfiguration driveMotorConfiguration = new TalonFXConfiguration().withCurrentLimits(currentLimits).withFeedback(feedbackConfigs).withSlot0(driveMotorSlot0);
    this.driveMotor.getConfigurator().apply(driveMotorConfiguration);
    this.driveMotor.setInverted(true);
    this.driveMotor.setNeutralMode(NeutralModeValue.Brake);

    // Boilerplate configuration for the turn motor to prevent issues from arriving due to cached values
    this.turnMotor = new REVSpark(turnPort, CANSparkMax.MotorType.kBrushless);

    do {
      System.out.println("\n\n\nInitializing REVSpark for swerve module " + niceName + "...\n\n\n");
      this.turnMotor.restoreFactoryDefaults();
      this.turnMotor.setInverted(true);
//    this.turnMotor.getEncoder().setPosition(0);
      while (Math.abs(this.turnMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle).getZeroOffset() - kencoderOffset) >= 0.0001) {
        this.turnMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle).setZeroOffset(kencoderOffset);
        System.out.println("\n\n\n\n\nWaiting for zero offset on swerve module " + niceName + "...\nExpected: " + kencoderOffset + ", Actual: " + this.turnMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle).getZeroOffset() + "\n\n\n\n\n");
        try {
          Thread.sleep(10);
        } catch (InterruptedException e) {}
      }
//      System.out.println("\n\n\n\n\n\nPOSITION BEFORE FACTOR ON " + niceName + ": " + this.turnMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle).getPosition());
      this.turnMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle).setPositionConversionFactor(2 * Math.PI);
//      System.out.println("POSITION AFTER FACTOR: " + this.turnMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle).getPosition() + "\n\n\n\n\n\n");
      this.turnMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle).setVelocityConversionFactor(2 * Math.PI / 60.0);
      this.turnMotor.setSmartCurrentLimit(Constants.Drivetrain.TURN_CURRENT_LIMIT);
      this.turnMotor.getEncoder().setPositionConversionFactor(2 * Math.PI / Constants.Drivetrain.TURN_GEAR_RATIO);
      this.turnMotor.getEncoder().setVelocityConversionFactor(2 * Math.PI / 60.0 / Constants.Drivetrain.TURN_GEAR_RATIO);
      this.turnMotor.burnFlash();
    } while (Math.abs(this.turnMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle).getPosition() - this.turnMotor.getEncoder().getPosition() % (2 * Math.PI)) > 0.05);

    this.turnMotor.getEncoder().setPosition(this.turnMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle).getPosition());

    // Configure PID values & configuration for rotation motor
    SparkPIDController turnPID = this.turnMotor.getPIDController();
    // FIXME: Does this need to somehow fix NEO error stuff?
    turnPID.setFeedbackDevice(((REVSpark.REVEncoder) this.turnMotor.getEncoder()).getRelative());
    turnPID.setPositionPIDWrappingEnabled(true);
    turnPID.setPositionPIDWrappingMinInput(Constants.Drivetrain.PID_MIN_INPUT);
    turnPID.setPositionPIDWrappingMaxInput(Constants.Drivetrain.PID_MAX_INPUT);
    turnPID.setP(Constants.Drivetrain.TURN_P);
    turnPID.setI(Constants.Drivetrain.TURN_I);
    turnPID.setD(Constants.Drivetrain.TURN_D);
    this.turnMotor.burnFlash();
  }

  /**
    * Routinely updates the target velocity & angle and sends debugging information to SmartDashboard
   **/
  @Override
  public void periodic() {
    // Ensure that we don't get an error reading
    this.lastTurnValue = Util.getSparkPosition(this.turnMotor.getSpark(), this.lastTurnValue);

    // Set the target angle and velocity for module movement
    this.turnMotor.getPIDController().setReference(this.targetAngle, CANSparkBase.ControlType.kPosition);
    this.driveMotor.setControl(this.driveMotorRequest.withVelocity(this.targetVelocity));

    // Output current values to SmartDashboard for debugging
    SmartDashboard.putNumber(this.niceName + " turn angle", this.getTurnAngle());
    SmartDashboard.putNumber(this.niceName + " turn velocity", this.turnMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber(this.niceName + " turn kencoder position", this.turnMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle).getPosition());
    SmartDashboard.putNumber(this.niceName + " drive velocity", this.driveMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber(this.niceName + " drive encoder position", this.driveMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber(this.niceName + " target angle", this.targetAngle);
    SmartDashboard.putNumber(this.niceName + " target velocity", this.targetVelocity);
    SmartDashboard.putNumber(this.niceName + " turn current", this.turnMotor.getOutputCurrent());
    SmartDashboard.putNumber(this.niceName + " drive current", this.driveMotor.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber(this.niceName + " turn encoder position", this.getTurnAngle());
    SmartDashboard.putNumber(this.niceName + " turn encoder position % 2pi", this.getTurnAngle() % (2 * Math.PI));
    SmartDashboard.putNumber(this.niceName + " drive meters per second", this.driveMotor.getVelocity().getValueAsDouble());
  }

  //  Gets best way to turn to an angle without doing an extra rotation
  public static double bestTurn(double targetAngle, double currentPosition) {
    double minimumTarget = Math.floor(currentPosition / (2 * Math.PI)) * 2 * Math.PI + targetAngle;  // Number of rotations converted to radians + target turn
    double greatestTarget = minimumTarget + 2 * Math.PI;  // Subtract one rotation

    // Return smallest difference angle
    return Math.abs(minimumTarget - currentPosition) < Math.abs(greatestTarget - currentPosition) ? minimumTarget : greatestTarget;
  }

  /**
    * Set the target module angle and velocity.
    * @param state A SwerveModuleState object containing the requested values
   **/
  public void setTarget(SwerveModuleState state) {
//    this.targetAngle = bestTurn(state.angle.getRadians(), this.getTurnAngle());
    this.targetAngle = state.angle.getRadians();
    this.targetVelocity = state.speedMetersPerSecond;
  }

  /**
    * Gets the full position and rotation of the swerve module.
    * @return A SwerveModulePosition object containing the position and rotation values.
   **/
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(this.driveMotor.getPosition().getValueAsDouble(), new Rotation2d(getTurnAngle()));
  }

  /**
    * Gets the approximate 1-directional distance travelled.
    * @return The distance in meters
   **/
  public double getPositionMeters() {
    return this.driveMotor.getPosition().getValueAsDouble() / Constants.Drivetrain.DRIVE_GEAR_RATIO * Constants.Drivetrain.WHEEL_DIAMETER * Math.PI;
  }

  /**
    * Gets the swerve module rotation in radians.
    * @return The swerve module rotation
   **/
  public double getTurnAngle() {
    return this.lastTurnValue;
  }

  /**
    * Gets information about the swerve module's current velocity and rotation.
    * @return A SwerveModuleState object containing the velocity and rotational values
   **/
  public SwerveModuleState getState() {
    return new SwerveModuleState(this.driveMotor.getVelocity().getValueAsDouble(), new Rotation2d(getTurnAngle()));
  }
}
