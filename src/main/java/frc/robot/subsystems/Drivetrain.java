package frc.robot.subsystems;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.commands.MoveCommand;
import lib.swerve.SwerveKinematicLimits;
import lib.swerve.SwerveSetpoint;
import lib.swerve.SwerveSetpointGenerator;

public class Drivetrain extends SubsystemBase {
  private static Drivetrain instance;
  private final ADIS16470_IMU gyro = new ADIS16470_IMU();

  private static SwerveModule frontLeft;
  private static SwerveModule frontRight;
  private static SwerveModule backLeft;
  private static SwerveModule backRight;

  private final static Translation2d[] modulePositions = {
    new Translation2d(Units.inchesToMeters(8.625), Units.inchesToMeters(11.875)),  // Front Left
    new Translation2d(Units.inchesToMeters(8.625), Units.inchesToMeters(-11.875)),  // Front Right
    new Translation2d(Units.inchesToMeters(-11.125), Units.inchesToMeters(11.875)),  // Back Left
    new Translation2d(Units.inchesToMeters(-11.125), Units.inchesToMeters(-11.875))  // Back Right
  };

  // Set the maximum movement limits for robot kinematics
  private final SwerveKinematicLimits limits = new SwerveKinematicLimits(Constants.Drivetrain.METERS_PER_SECOND, Constants.Drivetrain.ACCELERATION, Constants.Drivetrain.ROTATION_RADIANS_PER_SECOND);

  public static SwerveDriveKinematics swerveKinematics;
  private final SwerveDrivePoseEstimator positionEstimator;
  private SwerveSetpoint prevSetpoint;
  private boolean fieldRelative;

  /**
    * The drivetrain subsystem for robot movement
   **/
  private Drivetrain() {
    this.fieldRelative = true;

    this.gyro.calibrate();

    // Create the SwerveDriveKinematics object
    this.swerveKinematics = new SwerveDriveKinematics(
      new Translation2d(Units.inchesToMeters(8.625), Units.inchesToMeters(11.875)),  // Front Left
      new Translation2d(Units.inchesToMeters(8.625), Units.inchesToMeters(-11.875)),  // Front Right
      new Translation2d(Units.inchesToMeters(-11.125), Units.inchesToMeters(11.875)),  // Back Left
      new Translation2d(Units.inchesToMeters(-11.125), Units.inchesToMeters(-11.875))  // Back Right
    );

    // Initialize swerve modules
    this.frontLeft = new SwerveModule(Constants.Ports.FRONT_LEFT_DRIVE, Constants.Ports.FRONT_LEFT_TURN, Constants.Drivetrain.FRONT_LEFT_KENCODER_OFFSET, "front left");
    this.frontRight = new SwerveModule(Constants.Ports.FRONT_RIGHT_DRIVE, Constants.Ports.FRONT_RIGHT_TURN, Constants.Drivetrain.FRONT_RIGHT_KENCODER_OFFSET, "front right");
    this.backLeft = new SwerveModule(Constants.Ports.BACK_LEFT_DRIVE, Constants.Ports.BACK_LEFT_TURN, Constants.Drivetrain.BACK_LEFT_KENCODER_OFFSET, "back left");
    this.backRight = new SwerveModule(Constants.Ports.BACK_RIGHT_DRIVE, Constants.Ports.BACK_RIGHT_TURN, Constants.Drivetrain.BACK_RIGHT_KENCODER_OFFSET, "back right");

    // Set up position estimator (like SwerveDriveOdometry but with the ability to add data from PhotonVision)
    this.positionEstimator = new SwerveDrivePoseEstimator(
      this.swerveKinematics,
      Rotation2d.fromDegrees(this.gyro.getAngle()),
      this.getPositionArray(),
      new Pose2d(Constants.WIDTH / 2, Constants.LENGTH / 2, Rotation2d.fromDegrees(0)) // Starting Position
    );

    // FIXME: unclear
    this.prevSetpoint = new SwerveSetpoint(this.getSpeed(), swerveKinematics.toSwerveModuleStates(this.getSpeed()));

    // Configure autonomous routines
    AutoBuilder.configureHolonomic(
      this::getPosition,
      this::resetPosition,
      this::getSpeed,
      this::swerveDrive,
      new HolonomicPathFollowerConfig(
        Constants.Drivetrain.METERS_PER_SECOND,
        Units.inchesToMeters(Math.hypot(Constants.WIDTH, Constants.LENGTH) / 2),
        new ReplanningConfig()
      ),
      () -> Constants.alliance.isPresent() && Constants.alliance.get() == DriverStation.Alliance.Red,
      this
    );
  }

  /**
    * Returns an initialized class of Drivetrain if one exists, or create a new one if it doesn't (and return it).
    * @return The drivetrain
   **/
  public static Drivetrain getInstance() {
    return Drivetrain.instance == null ? Drivetrain.instance = new Drivetrain() : Drivetrain.instance;
  }

  /**
    * Reset the robot's field-relative position
    * @param position The new offset/position of the robot
   **/
  public void resetPosition(Pose2d position) {
    this.positionEstimator.resetPosition(
      Rotation2d.fromDegrees(gyro.getAngle()),
      this.getPositionArray(),
      new Pose2d(position.getX() + Constants.WIDTH / 2, position.getY() + Constants.LENGTH / 2, position.getRotation())
    );
  }

  /**
    * Returns the approximate field-relative position of the robot.
    * @return A Pose2d containing the robot's position state
   **/
  public Pose2d getPosition() {
    return this.positionEstimator.getEstimatedPosition();
  }

  /**
    * Gets the combined speed of the robot.
    * @return A ChassisSpeeds object containing information regarding the speed of the robot.
   **/
  public ChassisSpeeds getSpeed() {
    return this.swerveKinematics.toChassisSpeeds(
      this.frontLeft.getState(),
      this.frontRight.getState(),
      this.backLeft.getState(),
      this.backRight.getState()
    );
  }

  /**
    * Routinely updates the robot's field-relative position according to motor movement.
   **/
  @Override
  public void periodic() {
    this.positionEstimator.update(
      Rotation2d.fromDegrees(gyro.getAngle()),
      this.getPositionArray()
    );
//    this.positionEstimator.addVisionMeasurement(Pose2d position, double timestamp);
  }

  /**
    * Creates a position array containing the current position states of each swerve module.
    * @return An array containing the swerve module positions.
   **/
  private SwerveModulePosition[] getPositionArray() {
    return new SwerveModulePosition[] {
      this.frontLeft.getPosition(),
      this.frontRight.getPosition(),
      this.backLeft.getPosition(),
      this.backRight.getPosition()
    };
  }

  /**
    * Reset the robot's orientation.
    * @return A command that, when run, will reset the gyroscope's orientation.
   **/
  public Command resetGyro() {
    return Commands.run(this.gyro::reset);
  }

  /**
    * Move the robot based on a provided ChassisSpeeds value
    * @param movement The requested ChassisSpeeds
   **/
  private void swerveDrive(ChassisSpeeds movement) {
    SwerveSetpointGenerator swerveStateGenerator = new SwerveSetpointGenerator(this.swerveKinematics);
    movement = ChassisSpeeds.discretize(movement, Constants.LOOP_INTERVAL);
    SmartDashboard.putNumber("vx", movement.vxMetersPerSecond);
    SmartDashboard.putNumber("vy", movement.vyMetersPerSecond);
    SmartDashboard.putNumber("omega", movement.omegaRadiansPerSecond);
    this.prevSetpoint = swerveStateGenerator.generateSetpoint(this.limits, this.prevSetpoint, movement, Constants.LOOP_INTERVAL);

    SwerveModuleState[] moduleStates = this.prevSetpoint.mModuleStates;
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
  public void swerveDrive(double vx, double vy, double omega) {
    SmartDashboard.putNumber("drive vx", vx);
    SmartDashboard.putNumber("drive vy", vy);
    SmartDashboard.putNumber("drive omega", omega);
    SmartDashboard.putNumber("drive velocity", Math.hypot(vx, vy));
    SmartDashboard.putNumber("heading", this.gyro.getAngle());

    if (this.fieldRelative) {
      this.swerveDrive(ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, Rotation2d.fromDegrees(this.gyro.getAngle())));
    } else {
      this.swerveDrive(new ChassisSpeeds(vx, vy, omega));
    }
  }

  public Command toggleFieldRelative() {
    return this.runOnce(() -> this.fieldRelative = !this.fieldRelative);
  }

  public void setFieldRelative(boolean fieldRelative) {
    this.fieldRelative = fieldRelative;
  }

  public boolean getFieldRelative() {
    return this.fieldRelative;
  }

  /**
    * Move the robot to a specified field-relative position.
    * @param target The target field-centric position on the field.
   **/
  public void swerveDrive(Pose2d target) {
    CommandScheduler.getInstance().schedule(new MoveCommand(target, Constants.MoveCommand.ERROR));
  }

  public Command goStraight(double speed, double distance){
    return new WaitCommand(distance / speed)
            .deadlineWith(this.run(() -> swerveDrive(new ChassisSpeeds(0.0, speed, 0.0)))
            .andThen(() -> swerveDrive(0, 0, 0)));
  }
  /**
    *
    * swerveDrive but with Bezier curves and such
    * the better way to do things, but more complicated
    * work on this later
    *
    * @param target The target field-centric position on the field.
    *
   **/
//  public void swerveDrive(Pose2d target) {
//    Command path = AutoBuilder.pathfindThenFollowPath(
//      target,
//      new PathConstraints(3.0, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(720)),
//      Constants.Drivetrain.ROTATION_DELAY_METERS
//    );
//  }
}
