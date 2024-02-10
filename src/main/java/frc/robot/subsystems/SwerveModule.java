package frc.robot.subsystems;

import com.revrobotics.*;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveModule extends SubsystemBase {
  private CANSparkMax driveMotor;
  private CANSparkMax turnMotor;
  private String niceName;
  private SimpleMotorFeedforward feedforward;

  private volatile double targetAngle;
  private volatile double targetVelocity;

  public SwerveModule(int drivePort, int turnPort, double kencoderOffset, String niceName) {
    this.niceName = niceName;
    this.driveMotor = new CANSparkMax(drivePort, CANSparkMax.MotorType.kBrushless);
    this.driveMotor.setInverted(true);
    this.driveMotor.setSmartCurrentLimit(Constants.Drivetrain.DRIVE_CURRENT_LIMIT);
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
    this.turnMotor.getPIDController().setReference(targetAngle, CANSparkBase.ControlType.kPosition);
    this.driveMotor.setVoltage(feedforward.calculate(targetVelocity));

    SmartDashboard.putNumber(this.niceName + " turn angle", this.getTurnAngle());
    SmartDashboard.putNumber(this.niceName + " turn velocity", this.turnMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber(this.niceName + " turn kencoder position", this.turnMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle).getPosition());
    SmartDashboard.putNumber(this.niceName + " drive velocity", this.driveMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber(this.niceName + " drive encoder position", this.driveMotor.getEncoder().getPosition());
    SmartDashboard.putNumber(this.niceName + " target angle", this.targetAngle);
    SmartDashboard.putNumber(this.niceName + " target velocity", this.targetVelocity);
    SmartDashboard.putNumber(this.niceName + " turn current", this.turnMotor.getOutputCurrent());
    SmartDashboard.putNumber(this.niceName + " drive current", this.driveMotor.getOutputCurrent());
  }

  public void setTarget(SwerveModuleState state) {
    this.targetAngle = state.angle.getRadians();
    this.targetVelocity = state.speedMetersPerSecond;
  }

  public double getPositionMeters() {
    return this.driveMotor.getEncoder().getPosition() * Constants.Drivetrain.WHEEL_DIAMETER * Math.PI;
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
      this.driveMotor.getEncoder().getVelocity(),
      new Rotation2d(this.turnMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle).getPosition())
    );
  }
}
