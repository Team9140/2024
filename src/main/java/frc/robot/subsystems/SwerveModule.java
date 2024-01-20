package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
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

  private volatile SwerveState currentState;
  private volatile SwerveState targetState;

  public SwerveModule(int drivePort, int turnPort) {
    this.driveMotor = new CANSparkMax(drivePort, CANSparkMax.MotorType.kBrushless);
    this.turnMotor = new CANSparkMax(turnPort, CANSparkMax.MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    SwerveState nextSwerveState = currentState;
    switch (currentState) {
      case STARTUP:
        turnMotor.getAlternateEncoder(0);
        initalOffset = turnMotor.getAlternateEncoder(Constants.kencoderCountsPerRev).getPosition();
        break;
      case POSITION:
        break;
      case FAULT:
        break;
    }
  }
}
