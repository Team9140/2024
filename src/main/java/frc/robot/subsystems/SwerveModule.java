package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import frc.robot.Constants;

public class SwerveModule {
  private CANSparkMax driveMotor;
  private CANSparkMax turnMotor;

  public SwerveModule(int drivePort, int turnPort) {
    this.driveMotor = new CANSparkMax(drivePort, CANSparkMaxLowLevel.MotorType.kBrushless);
    this.turnMotor = new CANSparkMax(turnPort, CANSparkMaxLowLevel.MotorType.kBrushless);
  }
}