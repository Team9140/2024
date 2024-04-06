package frc.robot.subsystems;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import lib.swerve.SwerveKinematicLimits;
import lib.swerve.SwerveSetpoint;
import lib.swerve.SwerveSetpointGenerator;

import java.util.function.Supplier;

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
  private double linearMultiplier = 1.0;
  private double rotationalMultiplier = 1.0;
  private Field2d dashboardField = new Field2d();
//  private PIDController turnJoyPID = new PIDController(Constants.Drivetrain.TURN_JOY_P, Constants.Drivetrain.TURN_JOY_I, Constants.Drivetrain.TURN_JOY_D, Constants.LOOP_INTERVAL);

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
      new Translation2d(Constants.Drivetrain.FRONT_OFFSET, Constants.Drivetrain.LEFT_OFFSET),  // Front Left
      new Translation2d(Constants.Drivetrain.FRONT_OFFSET, Constants.Drivetrain.RIGHT_OFFSET),  // Front Right
      new Translation2d(Constants.Drivetrain.BACK_OFFSET, Constants.Drivetrain.LEFT_OFFSET),  // Back Left
      new Translation2d(Constants.Drivetrain.BACK_OFFSET, Constants.Drivetrain.RIGHT_OFFSET)  // Back Right
    );

    this.swerveStateGenerator = new SwerveSetpointGenerator(this.swerveKinematics);

    // Initialize swerve modules
    this.frontLeft = new SwerveModule(
      "front left",
      Constants.Ports.FRONT_LEFT_DRIVE,
      Constants.Ports.FRONT_LEFT_TURN,
      Constants.Drivetrain.FF.FLS,
      Constants.Drivetrain.FF.FLV,
      Constants.Drivetrain.FF.FLA,
      true
    );
    this.frontRight = new SwerveModule(
      "front right",
      Constants.Ports.FRONT_RIGHT_DRIVE,
      Constants.Ports.FRONT_RIGHT_TURN,
      Constants.Drivetrain.FF.FRS,
      Constants.Drivetrain.FF.FRV,
      Constants.Drivetrain.FF.FRA,
      false
    );
    this.backLeft = new SwerveModule(
      "back left",
      Constants.Ports.BACK_LEFT_DRIVE,
      Constants.Ports.BACK_LEFT_TURN,
      Constants.Drivetrain.FF.BLS,
      Constants.Drivetrain.FF.BLV,
      Constants.Drivetrain.FF.BLA,
      true
    );
    this.backRight = new SwerveModule(
      "back right",
      Constants.Ports.BACK_RIGHT_DRIVE,
      Constants.Ports.BACK_RIGHT_TURN,
      Constants.Drivetrain.FF.BRS,
      Constants.Drivetrain.FF.BRV,
      Constants.Drivetrain.FF.BRA,
      false
    );

    // Set up position estimator (like SwerveDriveOdometry but with the ability to add data from PhotonVision)
    this.positionEstimator = new SwerveDrivePoseEstimator(
      this.swerveKinematics,
      Rotation2d.fromDegrees(this.gyro.getAngle()),
      this.getPositionArray(),
      new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0)) // Starting Position
    );

    // Make setpoint values to the initial state
    this.prevSetpoint = new SwerveSetpoint(this.getChassisSpeeds(), swerveKinematics.toSwerveModuleStates(this.getChassisSpeeds()));

    SmartDashboard.putData("Field", this.dashboardField);
  }

  /**
    * Routinely updates the robot's field-relative position according to motor movement.
   **/
  @Override
  public void periodic() {
    SmartDashboard.putNumber("gyro angle", this.gyro.getAngle());
    SmartDashboard.putNumber("gyro velocity", this.getRotationalVelocity());

    this.positionEstimator.update(
      Rotation2d.fromDegrees(gyro.getAngle()),
      this.getPositionArray()
    );

    Pose2d position = this.getPosition();
    this.dashboardField.setRobotPose(new Pose2d(position.getX() + 0.8, position.getY(), position.getRotation()));
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
//    this.resetGyro(position.getRotation().getDegrees());
    this.gyro.setGyroAngle(this.gyro.getYawAxis(), position.getRotation().getDegrees());
    this.positionEstimator.resetPosition(
      Rotation2d.fromDegrees(gyro.getAngle()),
      this.getPositionArray(),
      position
    );
  }

  public Command waitUntilPositionCommand(double x, double y, double theta) {
    return new WaitUntilCommand(() -> {
      Pose2d position = this.getPosition();
      return Math.abs(position.getX() - x) < Constants.Auto.WAIT_UNTIL_POSITION_ERROR.getX()
        && Math.abs(position.getY() - y) < Constants.Auto.WAIT_UNTIL_POSITION_ERROR.getY()
        && Math.abs(position.getRotation().getDegrees() - theta) % (2 * Math.PI) < Constants.Auto.WAIT_UNTIL_POSITION_ERROR.getRotation().getDegrees();
    });
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
  public ChassisSpeeds getChassisSpeeds() {
    return this.swerveKinematics.toChassisSpeeds(
      this.frontLeft.getState(),
      this.frontRight.getState(),
      this.backLeft.getState(),
      this.backRight.getState()
    );
  }

  public Command setDriveMultipliers(double linear, double rotational) {
    return this.runOnce(() -> {
      this.linearMultiplier = linear;
      this.rotationalMultiplier = rotational;
    });
  }

  /**
    * Reset the robot's orientation.
    * @return A command that, when run, will reset the gyroscope's orientation.
   **/
  public Command resetGyro() {
//    return Commands.run(this.gyro::reset);
    return this.resetGyro(0.0);
  }
  public Command resetGyro(double degrees) {
    return this.runOnce(() -> this.gyro.setGyroAngle(this.gyro.getYawAxis(), degrees));
  }

  public double getLinearVelocity() {
    return this.getChassisSpeeds().vxMetersPerSecond;
  }
  public double getRotationalVelocity() {
    return Math.toRadians(this.gyro.getRate());
  }

  public double getLinearDistanceMeters() {
    return this.positionEstimator.getEstimatedPosition().getX();
  }

  /**
    * Move the robot based on a provided ChassisSpeeds value
    * @param movement The requested ChassisSpeeds
   **/
  public void swerveDrive(ChassisSpeeds movement) {
    movement = ChassisSpeeds.discretize(
      new ChassisSpeeds(movement.vxMetersPerSecond, movement.vyMetersPerSecond, movement.omegaRadiansPerSecond),
      Constants.LOOP_INTERVAL
    );

    SmartDashboard.putNumber("drive vx", movement.vxMetersPerSecond);
    SmartDashboard.putNumber("drive vy", movement.vyMetersPerSecond);
    SmartDashboard.putNumber("drive omega", movement.omegaRadiansPerSecond);

    this.prevSetpoint = this.swerveStateGenerator.generateSetpoint(this.limits, this.prevSetpoint, movement, Constants.LOOP_INTERVAL);

    this.frontLeft.setTarget(this.prevSetpoint.mModuleStates[0]);
    this.frontRight.setTarget(this.prevSetpoint.mModuleStates[1]);
    this.backLeft.setTarget(this.prevSetpoint.mModuleStates[2]);
    this.backRight.setTarget(this.prevSetpoint.mModuleStates[3]);
  }

  public Command swerveDrive(Supplier<Double> getLeftJoyY, Supplier<Double> getLeftJoyX, Supplier<Double> getRightJoyX) {
    return this.run(() -> {
      double vx = -MathUtil.applyDeadband(getLeftJoyY.get(), Constants.Drivetrain.DRIVE_DEADBAND) * Constants.Drivetrain.LINEAR_VELOCITY * this.linearMultiplier;  // Forward (front-to-back) movement
      double vy = -MathUtil.applyDeadband(getLeftJoyX.get(), Constants.Drivetrain.DRIVE_DEADBAND) * Constants.Drivetrain.LINEAR_VELOCITY * this.linearMultiplier;  // Horizontal (side-to-side) movement
      double rightJoystickX = MathUtil.applyDeadband(getRightJoyX.get(), Constants.Drivetrain.TURN_DEADBAND);
      double omega = -rightJoystickX * Math.abs(rightJoystickX) * Constants.Drivetrain.ROTATION_VELOCITY * this.rotationalMultiplier;  // Rotation (squared to make larger values more sensitive)

      if (this.fieldRelative) {
        this.swerveDrive(ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, Rotation2d.fromDegrees(this.gyro.getAngle())));
      } else {
        this.swerveDrive(new ChassisSpeeds(vx, vy, omega));
      }
    });
  }

  public void rotateSysId(Measure<Voltage> vtheta) {
    double v = -vtheta.magnitude();
    SignalLogger.writeDouble("voltage", v);
    this.swerveDrive(new ChassisSpeeds(0.0, 0.0, v));
  }

  public void straightSysId(Measure<Voltage> vx) {
    double v = vx.magnitude();
    SignalLogger.writeDouble("voltage", v);
    this.swerveDrive(new ChassisSpeeds(v, 0.0, 0.0));
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
