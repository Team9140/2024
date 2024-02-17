package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveModule extends SubsystemBase {
  // Various motors [clarification needed]
  private TalonFX driveMotorLeader;

  // Allows full use of 15% power [clarification needed]
  private VoltageOut driveMotorRequest;

  // Controller for the rotation motor
  private CANSparkMax turnMotor;

  // For making SmartDashboard values easily discernible
  private String niceName;

  // Used to calculate velocity to voltage
  private SimpleMotorFeedforward feedforward;

  // The requested target angle and velocity of the swerve module
  private volatile double targetAngle;
  private volatile double targetVelocity;

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
    this.driveMotorLeader = new TalonFX(drivePort, "moe");
    this.driveMotorLeader.setInverted(true);

    // Enables FOC (15% extra power) [clarification needed]
    this.driveMotorRequest = new VoltageOut(0).withEnableFOC(true);
    CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs().withStatorCurrentLimit(Constants.Drivetrain.DRIVE_CURRENT_LIMIT).withStatorCurrentLimitEnable(true);
    TalonFXConfiguration driveMotorConfiguration = new TalonFXConfiguration().withCurrentLimits(currentLimits);
    this.driveMotorLeader.getConfigurator().apply(driveMotorConfiguration);

    // Boilerplate configuration for the turn motor to prevent issues from arriving due to cached values
    this.turnMotor = new CANSparkMax(turnPort, CANSparkMax.MotorType.kBrushless);
    this.turnMotor.restoreFactoryDefaults();
    this.turnMotor.setInverted(true);
    this.turnMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle).setZeroOffset(kencoderOffset);
    this.turnMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle).setPositionConversionFactor(2 * Math.PI);
    this.turnMotor.setSmartCurrentLimit(Constants.Drivetrain.TURN_CURRENT_LIMIT);

    // Tool to convert requested velocity into voltage
    this.feedforward = new SimpleMotorFeedforward(Constants.Drivetrain.MODULE_S, Constants.Drivetrain.MODULE_V, Constants.Drivetrain.MODULE_A);

    // Configure PID values & configuration for rotation motor
    SparkPIDController turnPID = this.turnMotor.getPIDController();
    turnPID.setFeedbackDevice(this.turnMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle));
    turnPID.setPositionPIDWrappingEnabled(true);
    turnPID.setPositionPIDWrappingMinInput(Constants.Drivetrain.PID_MIN_INPUT);
    turnPID.setPositionPIDWrappingMaxInput(Constants.Drivetrain.PID_MAX_INPUT);
    turnPID.setP(Constants.Drivetrain.TURN_P);
    turnPID.setI(Constants.Drivetrain.TURN_I);
    turnPID.setD(Constants.Drivetrain.TURN_D);
    this.turnMotor.burnFlash();

    // Potentially removable, PID settings for drive motor
//    SparkPIDController drivePID = this.driveMotor.getPIDController();
//    drivePID.setFeedbackDevice(this.driveMotor.getEncoder());
//    turnPID.setP(Constants.Drivetrain.DRIVE_P);
//    turnPID.setI(Constants.Drivetrain.DRIVE_I);
//    turnPID.setD(Constants.Drivetrain.DRIVE_D);
  }

  /**
    * Routinely updates the target velocity & angle and sends debugging information to SmartDashboard
   **/
  @Override
  public void periodic() {
    // Set the target angle and velocity for module movement
    this.turnMotor.getPIDController().setReference(targetAngle, CANSparkBase.ControlType.kPosition);
    this.driveMotorLeader.setControl(driveMotorRequest.withOutput(feedforward.calculate(targetVelocity)));

    // Output current values to SmartDashboard for debugging
    SmartDashboard.putNumber(this.niceName + " turn angle", this.getTurnAngle());
    SmartDashboard.putNumber(this.niceName + " turn velocity", this.turnMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber(this.niceName + " turn kencoder position", this.turnMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle).getPosition());
    SmartDashboard.putNumber(this.niceName + " drive velocity", this.driveMotorLeader.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber(this.niceName + " drive encoder position", this.driveMotorLeader.getPosition().getValueAsDouble());
    SmartDashboard.putNumber(this.niceName + " target angle", this.targetAngle);
    SmartDashboard.putNumber(this.niceName + " target velocity", this.targetVelocity);
    SmartDashboard.putNumber(this.niceName + " turn current", this.turnMotor.getOutputCurrent());
    // I think this gets the current at the motor level [clarification needed]
    SmartDashboard.putNumber(this.niceName + " drive current", this.driveMotorLeader.getStatorCurrent().getValueAsDouble());
  }

  /**
    * Set the target module angle and velocity.
    * @param state A SwerveModuleState object containing the requested values
   **/
  public void setTarget(SwerveModuleState state) {
    this.targetAngle = state.angle.getRadians();
    this.targetVelocity = state.speedMetersPerSecond;
  }

  /**
   * Gets the full position and rotation of the swerve module.
   * @return A SwerveModulePosition object containing the position and rotation values.
   **/
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(this.getPositionMeters(), new Rotation2d(getTurnAngle()));
  }

  /**
    * Gets the approximate 1-directional distance travelled.
    * @return The distance in meters
   **/
  public double getPositionMeters() {
    return this.driveMotorLeader.getPosition().getValueAsDouble() * Constants.Drivetrain.WHEEL_DIAMETER * Math.PI;
  }

  /**
    * Gets the swerve module rotation in radians.
    * @return The swerve module rotation
   **/
  public double getTurnAngle() {
    return this.turnMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle).getPosition();
  }

  /**
    * Gets information about the swerve module's current velocity and rotation.
    * @return A SwerveModuleState object containing the velocity and rotational values
   **/
  public SwerveModuleState getState() {
    return new SwerveModuleState(this.driveMotorLeader.getVelocity().getValueAsDouble(), new Rotation2d(getTurnAngle()));
  }
}
