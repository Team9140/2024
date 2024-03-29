package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import lib.swerve.SwerveKinematicLimits;
import lib.swerve.SwerveSetpoint;
import lib.swerve.SwerveSetpointGenerator;

public class Drivetrain extends SubsystemBase {
  private static Drivetrain instance;
  private final ADIS16470_IMU gyro = new ADIS16470_IMU();

  private final SwerveModule frontLeft;
  private final SwerveModule frontRight;
  private final SwerveModule backLeft;
  private final SwerveModule backRight;

  // Set the maximum movement limits for robot kinematics
  private final SwerveKinematicLimits limits = new SwerveKinematicLimits(Constants.Drivetrain.LINEAR_VELOCITY, Constants.Drivetrain.ACCELERATION, Constants.Drivetrain.ROTATION_VELOCITY);

  public final SwerveDriveKinematics swerveKinematics;

  private final SwerveSetpointGenerator swerveStateGenerator;
  private final SwerveDrivePoseEstimator positionEstimator;
  private SwerveSetpoint prevSetpoint;
  private boolean fieldRelative = true;

  /**
   * Returns an initialized class of Drivetrain if one exists, or create a new one if it doesn't (and return it).
   * @return The drivetrain
   **/
  public static Drivetrain getInstance() {
    return Drivetrain.instance == null ? Drivetrain.instance = new Drivetrain() : Drivetrain.instance;
  }

  /**
    * The drivetrain subsystem for robot movement
   **/
  private Drivetrain() {
    this.gyro.calibrate();

    // Create the SwerveDriveKinematics object
    this.swerveKinematics = new SwerveDriveKinematics(
      new Translation2d(Constants.FRONT_OFFSET, Constants.LEFT_OFFSET),  // Front Left
      new Translation2d(Constants.FRONT_OFFSET, Constants.RIGHT_OFFSET),  // Front Right
      new Translation2d(Constants.BACK_OFFSET, Constants.LEFT_OFFSET),  // Back Left
      new Translation2d(Constants.BACK_OFFSET, Constants.RIGHT_OFFSET)  // Back Right
    );

    this.swerveStateGenerator = new SwerveSetpointGenerator(this.swerveKinematics);

    // Initialize swerve modules
    this.frontLeft = new SwerveModule(Constants.Ports.FRONT_LEFT_DRIVE, Constants.Ports.FRONT_LEFT_TURN, "front left");
    this.frontRight = new SwerveModule(Constants.Ports.FRONT_RIGHT_DRIVE, Constants.Ports.FRONT_RIGHT_TURN, "front right");
    this.backLeft = new SwerveModule(Constants.Ports.BACK_LEFT_DRIVE, Constants.Ports.BACK_LEFT_TURN, "back left");
    this.backRight = new SwerveModule(Constants.Ports.BACK_RIGHT_DRIVE, Constants.Ports.BACK_RIGHT_TURN, "back right");

    // Set up position estimator (like SwerveDriveOdometry but with the ability to add data from PhotonVision)
    this.positionEstimator = new SwerveDrivePoseEstimator(
      this.swerveKinematics,
      Rotation2d.fromDegrees(this.gyro.getAngle()),
      this.getPositionArray(),
      new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0)) // Starting Position
    );

    // FIXME: unclear
    this.prevSetpoint = new SwerveSetpoint(this.getSpeed(), swerveKinematics.toSwerveModuleStates(this.getSpeed()));
  }

  /**
    * Routinely updates the robot's field-relative position according to motor movement.
   **/
  @Override
  public void periodic() {
    SmartDashboard.putNumber("gyro angle", this.gyro.getAngle());

    this.positionEstimator.update(
      Rotation2d.fromDegrees(gyro.getAngle()),
      this.getPositionArray()
    );
//    this.positionEstimator.addVisionMeasurement(Pose2d position, double timestamp);
  }

  /**
    * Get the current angle of the robot
    * @return The gyro angle in degrees
   **/
  public double getGyroAngle() {
    return this.gyro.getAngle();
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
   * Reset the robot's field-relative position
   * @param position The new offset/position of the robot
   **/
  public void resetPosition(Pose2d position) {
    this.positionEstimator.resetPosition(
      Rotation2d.fromDegrees(gyro.getAngle()),
      this.getPositionArray(),
      position
    );
    // Causes odd dragging movement
//    this.gyro.setGyroAngle(this.gyro.getYawAxis(), position.getRotation().getDegrees());
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
  public void swerveDrive(ChassisSpeeds movement) {
    movement = ChassisSpeeds.discretize(movement, Constants.LOOP_INTERVAL);
    SmartDashboard.putNumber("drive vx", movement.vxMetersPerSecond);
    SmartDashboard.putNumber("drive vy", movement.vyMetersPerSecond);
    SmartDashboard.putNumber("drive omega", movement.omegaRadiansPerSecond);
    double EPSILON = 0.01;
    if (Math.abs(movement.vxMetersPerSecond) <= EPSILON && Math.abs(movement.vyMetersPerSecond) <= EPSILON && Math.abs(movement.omegaRadiansPerSecond) <= EPSILON) {
      this.prevSetpoint = swerveStateGenerator.generateSetpoint(this.limits, this.prevSetpoint, new ChassisSpeeds(0.0, 0.0, 0.0), Constants.LOOP_INTERVAL);
    } else {
      this.prevSetpoint = swerveStateGenerator.generateSetpoint(this.limits, this.prevSetpoint, movement, Constants.LOOP_INTERVAL);
    }

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

  /**
    * Sets the position f the relative encoder in the drive motors
    * @param position The position to set the motors to
   **/
  public void setPositionMeters(double position) {
    this.frontLeft.setPositionMeters(position);
    this.frontRight.setPositionMeters(position);
    this.backLeft.setPositionMeters(position);
    this.backRight.setPositionMeters(position);
  }

  /**
    * Goes forward
    * @param speed How fast to go forward
    * @param distance How far to go forward
    * @return A command that goes strait for the requested speed and distance
   **/
  public Command goStraight(double speed, double distance){
    return new WaitCommand(Math.abs(distance / speed))
      .deadlineWith(this.run(() -> swerveDrive(new ChassisSpeeds(speed, 0.0, 0.0)))
        .andThen(() -> swerveDrive(new ChassisSpeeds(0, 0, 0))));
//    this.setPositionMeters(0);
//    return this.run(() -> swerveDrive(new ChassisSpeeds(speed, 0.0, 0.0)))
//      .until(() -> (Math.abs(this.frontLeft.getPosition().distanceMeters) >= Math.abs(distance)))
//      .andThen(this.runOnce(() -> swerveDrive(new ChassisSpeeds(0, 0, 0))));
  }

  /**
    * Goes forward
    * @param vx_field Field relative x velocity
    * @param vy_field Field relative y velocity
    * @param distance how far to go
    * @return A command that goes strait for the requested velocities and distance
   **/
  public Command goStraight(double vx_field, double vy_field, double distance){
//    return new WaitCommand(distance / speed)
//      .deadlineWith(this.run(() -> swerveDrive(new ChassisSpeeds(speed, 0.0, 0.0)))
//        .andThen(() -> swerveDrive(new ChassisSpeeds(0, 0, 0)));
    this.setPositionMeters(0);
    return this.run(() -> swerveDrive(vx_field, vy_field, 0.0))
      .until(() -> (Math.abs(this.frontLeft.getPosition().distanceMeters) >= Math.abs(distance)))
      .andThen(this.runOnce(() -> swerveDrive(new ChassisSpeeds(0, 0, 0))));
  }

  /**
    * Toggles field relative mode.
    * @return A command that toggles field relative movement.
   **/
  public Command toggleFieldRelative() {
    return this.runOnce(() -> this.fieldRelative = !this.fieldRelative);
  }

  /**
    * Sets field relative
    * @param fieldRelative The requested state of field relative
   **/
  public void setFieldRelative(boolean fieldRelative) {
    this.fieldRelative = fieldRelative;
  }

  /**
    * @return the state of field relative
   **/
  public boolean getFieldRelative() {
    return this.fieldRelative;
  }
}
