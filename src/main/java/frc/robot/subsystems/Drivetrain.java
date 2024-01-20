package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import frc.robot.Constants;

public class Drivetrain {
  private static Drivetrain instance;
  private final WPI_PigeonIMU gyro = new WPI_PigeonIMU(Constants.Drivetrain.Gyro);

  public static SwerveDriveKinematics swerveKinematics;

  public static SwerveDriveOdometry swerveOdometry;

  Drivetrain() {
    swerveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(Constants.wheelBase / 2, Constants.trackWidth / 2),
            new Translation2d(Constants.wheelBase / 2, -Constants.trackWidth / 2),
            new Translation2d(-Constants.wheelBase / 2, Constants.trackWidth / 2),
            new Translation2d(-Constants.wheelBase / 2, -Constants.trackWidth / 2));
    Rotation2d rotation = gyro.getRotation2d();
    //    swerveOdometry = new SwerveDriveOdometry(swerveKinematics, rotation, new
    // SwerveModulePosition(0.0, rotation));
  }

  public Drivetrain getInstance() {
    return Drivetrain.instance == null
        ? Drivetrain.instance = new Drivetrain()
        : Drivetrain.instance;
  }
}
