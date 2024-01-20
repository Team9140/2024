package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

public class SwerveModule {
  private CANSparkMax driveMotor;
  private CANSparkMax turnMotor;

  public SwerveModule(int drivePort, int turnPort) {
    this.driveMotor = new CANSparkMax(drivePort, CANSparkMax.MotorType.kBrushless);
    this.turnMotor = new CANSparkMax(turnPort, CANSparkMax.MotorType.kBrushless);
  }
}
