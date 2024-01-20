package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

public class SwerveModule {
  private CANSparkMax driveMotor;
  private CANSparkMax turnMotor;

  private enum SwerveState {
    STARTUP,
    POSITION,
    FAULT
  }

  private SwerveState currentState;
  private SwerveState targetState;

  public SwerveModule(int drivePort, int turnPort) {
    this.driveMotor = new CANSparkMax(drivePort, CANSparkMax.MotorType.kBrushless);
    this.turnMotor = new CANSparkMax(turnPort, CANSparkMax.MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    SwerveState nextSwerveState = targetState;
    switch (nextSwerveState) {
      case STARTUP:
        turnMotor.getAlternateEncoder();
    }
  }
}
