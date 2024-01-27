package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  private static Drivetrain instance;
  private final WPI_PigeonIMU gyro = new WPI_PigeonIMU(Constants.Ports.GYRO);

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
    frontLeft =
        new SwerveModule(
            Constants.Ports.frontLeftDrivePort, Constants.Ports.frontLeftTurnPort);
    frontRight =
        new SwerveModule(
            Constants.Ports.frontRightDrivePort, Constants.Ports.frontRightTurnPort);
    backLeft =
        new SwerveModule(
            Constants.Ports.backLeftDrivePort, Constants.Ports.backLeftTurnPort);
    backRight =
        new SwerveModule(
            Constants.Ports.backRightDrivePort, Constants.Ports.backRightTurnPort);
    swerveOdometry =
        new SwerveDriveOdometry(
            swerveKinematics,
            rotation,
            new SwerveModulePosition[] {
              frontLeft.getPosition(),
              frontRight.getPosition(),
              backLeft.getPosition(),
              backRight.getPosition()
            });
  }

  public static Drivetrain getInstance() {
    return Drivetrain.instance == null
        ? Drivetrain.instance = new Drivetrain()
        : Drivetrain.instance;
  }

  public void swerveDrive(double leftY, double leftX, double rightX) {}

  
}
