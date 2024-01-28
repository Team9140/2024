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
  private double initialOffset = 0.0;
  private String niceName;
  private SimpleMotorFeedforward feedforward;

  private enum SwerveState {
    STARTUP,
    POSITION,
    FAULT
  }

  private SwerveState currentState;

  private volatile double targetAngle;
  private volatile double targetVelocity;

  public SwerveModule(int drivePort, int turnPort, double kencoderOffset, String niceName) {
    this.niceName = niceName;
    this.driveMotor = new CANSparkMax(drivePort, CANSparkMax.MotorType.kBrushless);
    this.driveMotor.setInverted(true);
    this.driveMotor.setSmartCurrentLimit(Constants.Drivetrain.DRIVE_CURRENT_LIMIT);
    this.turnMotor = new CANSparkMax(turnPort, CANSparkMax.MotorType.kBrushless);
    this.turnMotor.setInverted(true);
    this.turnMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle).setZeroOffset(kencoderOffset);
    this.turnMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle).setPositionConversionFactor(2 * Math.PI);
    this.turnMotor.setSmartCurrentLimit(Constants.Drivetrain.TURN_CURRENT_LIMIT);
    this.feedforward = new SimpleMotorFeedforward(Constants.Drivetrain.MODULE_S, Constants.Drivetrain.MODULE_V, Constants.Drivetrain.MODULE_A);
    this.currentState = SwerveState.STARTUP;

    SparkPIDController pid = this.turnMotor.getPIDController();
    pid.setPositionPIDWrappingEnabled(true);
    pid.setPositionPIDWrappingMinInput(Constants.Drivetrain.PID_MIN_INPUT);
    pid.setPositionPIDWrappingMaxInput(Constants.Drivetrain.PID_MAX_INPUT);
    pid.setP(Constants.Drivetrain.TURN_P);
    pid.setI(Constants.Drivetrain.TURN_I);
    pid.setD(Constants.Drivetrain.TURN_D);
  }

  public SwerveModule(int drivePort, int turnPort, double kencoderOffset) {
    this(drivePort, turnPort, kencoderOffset, "drive " + drivePort + " turn " + turnPort);
  }

  @Override
  public void periodic() {
    switch (currentState) {
      case STARTUP:
        SparkRelativeEncoder turnEncoder = (SparkRelativeEncoder) this.turnMotor.getEncoder();
        turnEncoder.setPositionConversionFactor(Constants.Drivetrain.positionConversionFactor);
        this.turnMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle).setPositionConversionFactor(Constants.Drivetrain.positionConversionFactor);
        turnEncoder.setPosition(this.turnMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle).getPosition());
        SparkRelativeEncoder driveEncoder = (SparkRelativeEncoder) this.driveMotor.getEncoder();
        driveEncoder.setPositionConversionFactor(Constants.Drivetrain.positionConversionFactor);
        currentState = SwerveState.POSITION;
        break;
      case POSITION:
        this.turnMotor.getPIDController().setReference(targetAngle, CANSparkBase.ControlType.kPosition);
        this.driveMotor.setVoltage(feedforward.calculate(targetVelocity));
        break;
      case FAULT:
        break;
    }

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

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
      this.driveMotor.getEncoder().getPosition() * Constants.Drivetrain.driveWheelDiameter / 2,
      new Rotation2d(this.turnMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle).getPosition())
    );
  }

  public double getTurnAngle() {
    return this.turnMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle).getPosition();
  }
}
