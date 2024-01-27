package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveModule extends SubsystemBase {
  private CANSparkMax driveMotor;
  private CANSparkMax turnMotor;
  private SimpleMotorFeedforward feedforward;
  private double initalOffset = 0.0;

  private enum SwerveState {
    STARTUP,


    POSITION,
    FAULT
  }

  private SwerveState currentState;

  private volatile int targetAngle;
  private volatile int targetVelocity;

  public SwerveModule(int drivePort, int turnPort) {
    this.driveMotor = new CANSparkMax(drivePort, CANSparkMax.MotorType.kBrushless);
    this.turnMotor = new CANSparkMax(turnPort, CANSparkMax.MotorType.kBrushless);
    this.feedforward = new SimpleMotorFeedforward(ks, kv, ka);
    this.currentState = SwerveState.STARTUP;
  }

  @Override
  public void periodic() {
    SwerveState nextSwerveState = currentState;
    switch (currentState) {
      case STARTUP:
        turnMotor.getAlternateEncoder(0);
        initalOffset =
            turnMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle).getPosition();
        break;
      case POSITION:
            turnMotor.getPIDController().setReference(targetAngle, ControlType.kPosition);
        break;
      case FAULT:
        break;
    }
  }
}
