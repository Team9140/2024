package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    swerveKinematics = new SwerveDriveKinematics(
      new Translation2d(Constants.wheelBase / 2, Constants.trackWidth / 2),  // Front Left
      new Translation2d(Constants.wheelBase / 2, -Constants.trackWidth / 2),  // Front Right
      new Translation2d(-Constants.wheelBase / 2, Constants.trackWidth / 2),  // Back Left
      new Translation2d(-Constants.wheelBase / 2, -Constants.trackWidth / 2)  // Back Right
    );
    Rotation2d rotation = gyro.getRotation2d();
    this.frontLeft = new SwerveModule(Constants.Ports.frontLeftDrivePort, Constants.Ports.frontLeftTurnPort, "front left");
    this.frontRight = new SwerveModule(Constants.Ports.frontRightDrivePort, Constants.Ports.frontRightTurnPort, "front right");
    this.backLeft = new SwerveModule(Constants.Ports.backLeftDrivePort, Constants.Ports.backLeftTurnPort, "back left");
    this.backRight =new SwerveModule(Constants.Ports.backRightDrivePort, Constants.Ports.backRightTurnPort, "back right");
    this.swerveOdometry = new SwerveDriveOdometry(
      swerveKinematics,
      rotation,
      new SwerveModulePosition[] {
        this.frontLeft.getPosition(),
        this.frontRight.getPosition(),
        this.backLeft.getPosition(),
        this.backRight.getPosition()
      }
    );
  }

  public static Drivetrain getInstance() {
    return Drivetrain.instance == null ? Drivetrain.instance = new Drivetrain() : Drivetrain.instance;
  }

  /**
    * Move the robot based on the current controller joystick values
    * @param vx Forward velocity (positive is forward)
    * @param vy Horizontal velocity (positive is left)
    * @param omega Rotational velocity (positive is ccw)
   **/
  public void swerveDrive(double vx, double vy, double omega) {
    vx = MathUtil.applyDeadband(vx, Constants.Drivetrain.DEADBAND);
    vy = MathUtil.applyDeadband(vy, Constants.Drivetrain.DEADBAND);
    omega = MathUtil.applyDeadband(omega, Constants.Drivetrain.DEADBAND);

    SmartDashboard.putNumber("drive vx", vx);
    SmartDashboard.putNumber("drive vy", vy);
    SmartDashboard.putNumber("drive omega", omega);

    ChassisSpeeds movement = new ChassisSpeeds(vx, vy, omega);

    SwerveModuleState[] moduleStates = swerveKinematics.toSwerveModuleStates(
      movement, new Translation2d(Units.inchesToMeters(1.5), 0));
    moduleStates[0] = SwerveModuleState.optimize(moduleStates[0], new Rotation2d(frontLeft.getTurnAngle()));
    moduleStates[1] = SwerveModuleState.optimize(moduleStates[1], new Rotation2d(frontRight.getTurnAngle()));
    moduleStates[2] = SwerveModuleState.optimize(moduleStates[2], new Rotation2d(backLeft.getTurnAngle()));
    moduleStates[3] = SwerveModuleState.optimize(moduleStates[3], new Rotation2d(backRight.getTurnAngle()));

    frontLeft.setTarget(moduleStates[0]);
    frontRight.setTarget(moduleStates[1]);
    backLeft.setTarget(moduleStates[2]);
    backRight.setTarget(moduleStates[3]);
  }
}
