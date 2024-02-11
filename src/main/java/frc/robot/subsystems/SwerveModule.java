package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.*;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveModule extends SubsystemBase {
  // Various motors
  private TalonFX driveMotorLeader;
  // Allows full use of 15% power
  private VoltageOut driveMotorRequest;
  private CANSparkMax turnMotor;

  // For logging prettiness
  private String niceName;

  // Used to calculate velocity to voltage
  private SimpleMotorFeedforward feedforward;

  private volatile double targetAngle;
  private volatile double targetVelocity;

  public SwerveModule(int drivePort, int turnPort, double kencoderOffset, String niceName) {
    this.niceName = niceName;

    // TalonFX doesn't use RIO canbus, it uses its own
    this.driveMotorLeader = new TalonFX(drivePort, "moe");
    this.driveMotorLeader.setInverted(true);
    // Enables FOC (15% extra power)
    this.driveMotorRequest = new VoltageOut(0).withEnableFOC(true);
    CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs().withStatorCurrentLimit(Constants.Drivetrain.DRIVE_CURRENT_LIMIT).withStatorCurrentLimitEnable(true);
    TalonFXConfiguration driveMotorConfiguration = new TalonFXConfiguration().withCurrentLimits(currentLimits);
    this.driveMotorLeader.getConfigurator().apply(driveMotorConfiguration);

    // Configures turn motor
    this.turnMotor = new CANSparkMax(turnPort, CANSparkMax.MotorType.kBrushless);
    this.turnMotor.restoreFactoryDefaults();
    this.turnMotor.setInverted(true);
    this.turnMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle).setZeroOffset(kencoderOffset);
    this.turnMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle).setPositionConversionFactor(2 * Math.PI);
    this.turnMotor.setSmartCurrentLimit(Constants.Drivetrain.TURN_CURRENT_LIMIT);

    this.feedforward = new SimpleMotorFeedforward(Constants.Drivetrain.MODULE_S, Constants.Drivetrain.MODULE_V, Constants.Drivetrain.MODULE_A);

    SparkPIDController turnPID = this.turnMotor.getPIDController();
    turnPID.setFeedbackDevice(this.turnMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle));
    turnPID.setPositionPIDWrappingEnabled(true);
    turnPID.setPositionPIDWrappingMinInput(Constants.Drivetrain.PID_MIN_INPUT);
    turnPID.setPositionPIDWrappingMaxInput(Constants.Drivetrain.PID_MAX_INPUT);
    turnPID.setP(Constants.Drivetrain.TURN_P);
    turnPID.setI(Constants.Drivetrain.TURN_I);
    turnPID.setD(Constants.Drivetrain.TURN_D);

//    SparkPIDController drivePID = this.driveMotor.getPIDController();
//    drivePID.setFeedbackDevice(this.driveMotor.getEncoder());
//    turnPID.setP(Constants.Drivetrain.DRIVE_P);
//    turnPID.setI(Constants.Drivetrain.DRIVE_I);
//    turnPID.setD(Constants.Drivetrain.DRIVE_D);

    this.turnMotor.burnFlash();
  }

  @Override
  public void periodic() {
    // Sets angles and velocities
    this.turnMotor.getPIDController().setReference(targetAngle, CANSparkBase.ControlType.kPosition);
    this.driveMotorLeader.setControl(driveMotorRequest.withOutput(feedforward.calculate(targetVelocity)));

    SmartDashboard.putNumber(this.niceName + " turn angle", this.getTurnAngle());
    SmartDashboard.putNumber(this.niceName + " turn velocity", this.turnMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber(this.niceName + " turn kencoder position", this.turnMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle).getPosition());
    SmartDashboard.putNumber(this.niceName + " drive velocity", this.driveMotorLeader.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber(this.niceName + " drive encoder position", this.driveMotorLeader.getPosition().getValueAsDouble());
    SmartDashboard.putNumber(this.niceName + " target angle", this.targetAngle);
    SmartDashboard.putNumber(this.niceName + " target velocity", this.targetVelocity);
    SmartDashboard.putNumber(this.niceName + " turn current", this.turnMotor.getOutputCurrent());
    // I think this gets the current at the motor level
    SmartDashboard.putNumber(this.niceName + " drive current", this.driveMotorLeader.getStatorCurrent().getValueAsDouble());
  }

  public void setTarget(SwerveModuleState state) {
    this.targetAngle = state.angle.getRadians();
    this.targetVelocity = state.speedMetersPerSecond;
  }

  public double getPositionMeters() {
    return this.driveMotorLeader.getPosition().getValueAsDouble() * Constants.Drivetrain.WHEEL_DIAMETER * Math.PI;
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
      this.getPositionMeters(),
      new Rotation2d(this.turnMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle).getPosition())
    );
  }

  public double getTurnAngle() {
    return this.turnMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle).getPosition();
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(
      this.driveMotorLeader.getVelocity().getValueAsDouble(),
      new Rotation2d(this.turnMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle).getPosition())
    );
  }
}
