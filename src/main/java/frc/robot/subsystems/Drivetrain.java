package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  private static Drivetrain instance;
//  private final ADIS16470_IMU gyro = new ADIS16470_IMU();
  private final ADXRS450_Gyro gyro = new ADXRS450_Gyro();

  private static SwerveModule frontLeft;
  private static SwerveModule frontRight;
  private static SwerveModule backLeft;
  private static SwerveModule backRight;

  public static SwerveDriveKinematics swerveKinematics;

  public static SwerveDriveOdometry swerveOdometry;

  private Drivetrain() {
    this.gyro.calibrate();

    this.swerveKinematics = new SwerveDriveKinematics(
      new Translation2d(Units.inchesToMeters(8.625), Units.inchesToMeters(11.875)),  // Front Left
      new Translation2d(Units.inchesToMeters(8.625), Units.inchesToMeters(-11.875)),  // Front Right
      new Translation2d(Units.inchesToMeters(-11.125), Units.inchesToMeters(11.875)),  // Back Left
      new Translation2d(Units.inchesToMeters(-11.125), Units.inchesToMeters(-11.875))  // Back Right
    );

    this.frontLeft = new SwerveModule(Constants.Ports.FRONT_LEFT_DRIVE, Constants.Ports.FRONT_LEFT_TURN, Constants.Drivetrain.FRONT_LEFT_KENCODER_OFFSET, "front left");
    this.frontRight = new SwerveModule(Constants.Ports.FRONT_RIGHT_DRIVE, Constants.Ports.FRONT_RIGHT_TURN, Constants.Drivetrain.FRONT_RIGHT_KENCODER_OFFSET, "front right");
    this.backLeft = new SwerveModule(Constants.Ports.BACK_LEFT_DRIVE, Constants.Ports.BACK_LEFT_TURN, Constants.Drivetrain.BACK_LEFT_KENCODER_OFFSET, "back left");
    this.backRight = new SwerveModule(Constants.Ports.BACK_RIGHT_DRIVE, Constants.Ports.BACK_RIGHT_TURN, Constants.Drivetrain.BACK_RIGHT_KENCODER_OFFSET, "back right");
    this.swerveOdometry = new SwerveDriveOdometry(
      swerveKinematics,
//      new Rotation2d(gyro.getAngle()),  // ADIS16470_IMU
      gyro.getRotation2d(),
      new SwerveModulePosition[] {
        this.frontLeft.getPosition(),
        this.frontRight.getPosition(),
        this.backLeft.getPosition(),
        this.backRight.getPosition()
      }
    );

    AutoBuilder.configureHolonomic(
      this::getPosition,
      this::resetPosition,
      this::getSpeed,
      this::swerveDrive,
      new HolonomicPathFollowerConfig(
        Constants.Drivetrain.FORWARD_METERS_PER_SECOND,
        Units.inchesToMeters(Math.hypot(Constants.WIDTH, Constants.LENGTH) / 2),
        new ReplanningConfig()
      ),
      () -> Constants.alliance.isPresent() && Constants.alliance.get() == DriverStation.Alliance.Red,
      this
    );
  }

  public static Drivetrain getInstance() {
    return Drivetrain.instance == null ? Drivetrain.instance = new Drivetrain() : Drivetrain.instance;
  }

  public Pose2d getPosition() {
    return this.swerveOdometry.getPoseMeters();
  }

  private void resetPosition(Pose2d position) {
//    this.swerveOdometry.resetPosition(new Rotation2d(gyro.getAngle()), new SwerveModulePosition[] {  // ADIS16470_IMU
    this.swerveOdometry.resetPosition(gyro.getRotation2d(), new SwerveModulePosition[] {  // Trash Gyro
      new SwerveModulePosition(this.frontLeft.getPositionMeters(), new Rotation2d(this.frontLeft.getTurnAngle())),
      new SwerveModulePosition(this.frontRight.getPositionMeters(), new Rotation2d(this.frontRight.getTurnAngle())),
      new SwerveModulePosition(this.backLeft.getPositionMeters(), new Rotation2d(this.backLeft.getTurnAngle())),
      new SwerveModulePosition(this.backRight.getPositionMeters(), new Rotation2d(this.backRight.getTurnAngle())),
    }, position);
  }

  public ChassisSpeeds getSpeed() {
    return this.swerveKinematics.toChassisSpeeds(
      this.frontLeft.getState(),
      this.frontRight.getState(),
      this.backLeft.getState(),
      this.backRight.getState()
    );
  }

  @Override
  public void periodic() {
    this.swerveOdometry.update(
//      new Rotation2d(gyro.getAngle()),  // ADIS16470_IMU
      gyro.getRotation2d(),  // PigeonIMU
      new SwerveModulePosition[] {
        this.frontLeft.getPosition(),
        this.frontRight.getPosition(),
        this.backLeft.getPosition(),
        this.backRight.getPosition()
      }
    );
  }

  public Command resetGyro() {
    return Commands.run(() -> this.gyro.reset());
  }

  /**
    * Move the robot based on a provided ChassisSpeeds value
    * @param movement The requested ChassisSpeeds
   **/
  private void swerveDrive(ChassisSpeeds movement) {
    SwerveModuleState[] moduleStates = swerveKinematics.toSwerveModuleStates(movement, new Translation2d(Units.inchesToMeters(1.5), 0));
    moduleStates[0] = SwerveModuleState.optimize(moduleStates[0], new Rotation2d(this.frontLeft.getTurnAngle()));
    moduleStates[1] = SwerveModuleState.optimize(moduleStates[1], new Rotation2d(this.frontRight.getTurnAngle()));
    moduleStates[2] = SwerveModuleState.optimize(moduleStates[2], new Rotation2d(this.backLeft.getTurnAngle()));
    moduleStates[3] = SwerveModuleState.optimize(moduleStates[3], new Rotation2d(this.backRight.getTurnAngle()));

    this.frontLeft.setTarget(moduleStates[0]);
    this.frontRight.setTarget(moduleStates[1]);
    this.backLeft.setTarget(moduleStates[2]);
    this.backRight.setTarget(moduleStates[3]);
  }

  /**
    * Move the robot based on the current controller joystick values
    * @param vx Forward velocity (positive is forward)
    * @param vy Horizontal velocity (positive is left)
    * @param omega Rotational velocity (positive is ccw)
   **/
  public void swerveDrive(double vx, double vy, double omega, boolean fieldRelative) {
    SmartDashboard.putNumber("drive vx", vx);
    SmartDashboard.putNumber("drive vy", vy);
    SmartDashboard.putNumber("drive omega", omega);
    SmartDashboard.putNumber("drive velocity", Math.hypot(vx, vy));

    if (fieldRelative) {
      this.swerveDrive(ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, this.gyro.getRotation2d()));
    } else {
      this.swerveDrive(new ChassisSpeeds(vx, vy, omega));
    }
  }
}
