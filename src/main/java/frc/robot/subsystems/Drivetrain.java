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

  private static SwerveModule frontLeft;
  private static SwerveModule frontRight;

  private static SwerveModule backLeft;

  private static SwerveModule backRight;

  public static SwerveDriveKinematics swerveKinematics;

  public static SwerveDriveOdometry swerveOdometry;

  private Drivetrain() {
    swerveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(Constants.wheelBase / 2, Constants.trackWidth / 2),
            new Translation2d(Constants.wheelBase / 2, -Constants.trackWidth / 2),
            new Translation2d(-Constants.wheelBase / 2, Constants.trackWidth / 2),
            new Translation2d(-Constants.wheelBase / 2, -Constants.trackWidth / 2));
    Rotation2d rotation = gyro.getRotation2d();
    frontLeft = new SwerveModule(Constants.Drivetrain.frontLeftDrivePort, Constants.Drivetrain.frontLeftTurnPort);
    frontRight = new SwerveModule(Constants.Drivetrain.frontRightDrivePort, Constants.Drivetrain.frontRightTurnPort);
    backLeft = new SwerveModule(Constants.Drivetrain.backLeftDrivePort, Constants.Drivetrain.backLeftTurnPort);
    backRight = new SwerveModule(Constants.Drivetrain.backRightDrivePort, Constants.Drivetrain.backRightTurnPort);
    //    swerveOdometry = new SwerveDriveOdometry(swerveKinematics, rotation, new
    // SwerveModulePosition(0.0, rotation));
  }

  public Drivetrain getInstance() {
    return Drivetrain.instance == null
        ? Drivetrain.instance = new Drivetrain()
        : Drivetrain.instance;
  }
}
