package frc.robot.subsystems;

import com.revrobotics.*;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveModule extends SubsystemBase {
  private CANSparkMax driveMotor;
  private CANSparkMax turnMotor;
  private double initalOffset = 0.0;

  private SparkRelativeEncoder turnEncoder;
  private SparkRelativeEncoder driveEncoder;

  private SimpleMotorFeedforward feedforward;

  private enum SwerveState {
    STARTUP,


    POSITION,
    FAULT
  }

  private SwerveState currentState;

  private volatile double targetAngle;
  private volatile double targetVelocity;

  public SwerveModule(int drivePort, int turnPort) {
    double ks = 0;
    double kv = 0;
    double ka = 0;
    this.driveMotor = new CANSparkMax(drivePort, CANSparkMax.MotorType.kBrushless);
    this.turnMotor = new CANSparkMax(turnPort, CANSparkMax.MotorType.kBrushless);
    this.feedforward = new SimpleMotorFeedforward(ks, kv, ka);
    this.currentState = SwerveState.STARTUP;
  }

  @Override
  public void periodic() {
    switch (currentState) {
      case STARTUP:
        this.turnEncoder = (SparkRelativeEncoder) this.turnMotor.getEncoder();
        this.turnEncoder.setPosition(
            this.turnMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle).getPosition());
        this.turnEncoder.setPositionConversionFactor(Constants.Drivetrain.positionConversionFactor);
        this.driveEncoder = (SparkRelativeEncoder) this.driveMotor.getEncoder();
        this.driveEncoder.setPositionConversionFactor(
            Constants.Drivetrain.positionConversionFactor);
        this.currentState = SwerveState.POSITION;
        break;
      case POSITION:
        this.turnMotor
            .getPIDController()
            .setReference(targetAngle, CANSparkBase.ControlType.kPosition);
        this.driveMotor.setVoltage(feedforward.calculate(targetVelocity));
        break;
      case FAULT:
        break;
    }
  }

  public void setTarget(double targetAngle, double targetVelocity) {
    this.targetAngle = targetAngle % Constants.Drivetrain.positionConversionFactor;
    this.targetVelocity = targetVelocity;
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        this.driveEncoder.getPosition() * Constants.Drivetrain.driveWheelDiameter / 2,
        new Rotation2d(turnEncoder.getPosition()));
  }
}
