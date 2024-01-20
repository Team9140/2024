package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveModule extends SubsystemBase {
  private CANSparkMax driveMotor;
  private CANSparkMax turnMotor;
  private double initalOffset;

  private enum SwerveState {
    STARTUP,
    POSITION,
    FAULT
  }

  private SwerveState currentState = SwerveState.STARTUP;

  private volatile int targetAngle;
  private volatile int targetVelocity;

  public SwerveModule(int drivePort, int turnPort) {
    this.driveMotor = new CANSparkMax(drivePort, CANSparkMax.MotorType.kBrushless);
    this.turnMotor = new CANSparkMax(turnPort, CANSparkMax.MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    SwerveState nextSwerveState = currentState;
    switch (currentState) {
      case STARTUP:
        turnMotor.getAlternateEncoder(Constants.kencoderCountsPerRev);
        initalOffset = turnMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle).getPosition();
        break;
      case POSITION:
//        turnMotor.getPIDController().setReference()
        break;
      case FAULT:
        break;
    }
  }
}