package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkRelativeEncoder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveModule extends SubsystemBase {
  private CANSparkMax driveMotor;
  private CANSparkMax turnMotor;
  private double initialOffset = 0.0;

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
        SparkRelativeEncoder turnEncoder = (SparkRelativeEncoder) this.turnMotor.getEncoder();
        turnEncoder.setPosition(this.turnMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle).getPosition());
        turnEncoder.setPositionConversionFactor(Constants.Drivetrain.positionConversionFactor);

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
  }

  public void setTarget(SwerveModuleState state) {
    this.targetAngle = state.angle.getRadians();
    this.targetVelocity = state.speedMetersPerSecond;
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
      this.driveMotor.getEncoder().getPosition() * Constants.Drivetrain.driveWheelDiameter / 2,
      new Rotation2d(this.turnMotor.getEncoder().getPosition())
    );
  }

  public double getTurnAngle() {
    return this.turnMotor.getEncoder().getPosition();
  }
}
