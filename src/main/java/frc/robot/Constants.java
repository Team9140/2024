// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

public final class Constants {
  public static final double LOOP_INTERVAL = 0.010;  // Periodic interval delay time FIXME: unknown units
  public static final double TRACK_WIDTH = Units.inchesToMeters(23.75);  // Horizontal (side-to-side) distance between wheels
  public static final double WHEEL_BASE = Units.inchesToMeters(20.75);  // Front-to-back distance between wheels
  public static final double scoringRange = 120.0;  // FIXME: Unknown

  // Full field length
  public static final double FIELD_LENGTH = 16.54;  // The length of the field in meters

  // Full-body dimensions
  public static final int WIDTH = 29;  // Inches, side-to-side width
  public static final int LENGTH = 29;  // Inches, front-to-back length

  // Size of the field
  //  public static final double fieldx = Units.inchesToMeters(501);
  //  public static final double fieldy = Units.inchesToMeters(323.28);
  public static final class Drivetrain {
    public static final double TURN_GEAR_RATIO = 150.0 / 7.0;
    public static final double DRIVE_GEAR_RATIO = (50.0 / 16.0) * (16.0 / 28.0) * (45.0 / 15.0);
    public static final double WHEEL_DIAMETER = 4;  // Diameter of each wheel in inches
    public static final double DRIVE_DEADBAND = 0.10;  // Input deadband value for left joystick
    public static final double TURN_DEADBAND = 0.15;  // Input deadband value for right joystick

    // Max linear and rotational speeds
    public static final double METERS_PER_SECOND = Units.feetToMeters(19);  // ft/s max velocity
    public static final double ACCELERATION = Units.feetToMeters(80);  // ft/s^2 max acceleration
    public static final double ROTATION_RADIANS_PER_SECOND = (2 * Math.PI) * 2;  // 2 complete rotations per second (720 deg/s)


    // Drive motor feedforward values
    public static final double MODULE_S = 0;
    public static final double MODULE_V = 2.43219076;
    public static final double MODULE_A = 0;


    // Rotation offset for swerve module kencoders
    public static final double FRONT_LEFT_KENCODER_OFFSET = 0.605;
    public static final double FRONT_RIGHT_KENCODER_OFFSET = 0.795;
    public static final double BACK_LEFT_KENCODER_OFFSET = 0.320;
    public static final double BACK_RIGHT_KENCODER_OFFSET = 0.375;


    // PID values for the turn motor
    public static final double TURN_P = 1.97;
    public static final double TURN_I = 0.0;
    public static final double TURN_D = 0.115;


    // PID values for the drive motor
    public static final double DRIVE_P = 6.6544;
    public static final double DRIVE_I = 0.0;
    public static final double DRIVE_D = 4.833;


    // Electric current limits for the swerve modules
    public static final int DRIVE_CURRENT_LIMIT = 60;
    public static final int TURN_CURRENT_LIMIT = 30;


    // PID wrapping values for the turn motor
    public static final double PID_MIN_INPUT = 0.0;
    public static final double PID_MAX_INPUT = 2 * Math.PI;

    public static final double TURN_REGULAR_NOBOOST = 70.0 / 100.0;  // Percent requested rotation without boost enabled
    public static final double DRIVE_REGULAR_NOBOOST = 70.0 / 100.0;  // Percent requested drive without boost enabled

    // Distance to travel before rotation is attempted
//    public static final double ROTATION_DELAY_METERS = 1.0;
  }

  public static class Ports {
    public static final int INPUT_CONTROLLER = 0;  // Xbox Controller
    public static final String CAMERA = "ProblemCamera";  // PhotonVision camera ID

    // Ports for CANSparkMax motor controllers
    public static final int FRONT_LEFT_DRIVE = 1;
    public static final int FRONT_LEFT_TURN = 4;
    public static final int FRONT_RIGHT_DRIVE = 0;
    public static final int FRONT_RIGHT_TURN = 9;
    public static final int BACK_LEFT_DRIVE = 3;
    public static final int BACK_LEFT_TURN = 1;
    public static final int BACK_RIGHT_DRIVE = 2;
    public static final int BACK_RIGHT_TURN = 7;

    public static final String CTRE_CANBUS = "jama";
    public static final int ARM_MOTOR = 19;
    public static final int BOTTOM_SHOOTER = 20;
    public static final int TOP_SHOOTER = 21;
    public static final int ARM_FEEDER = 22;

    public static final int FRONT_LEFT_INTAKE = 32;
    public static final int FRONT_RIGHT_INTAKE = 31;
    public static final int BACK_INTAKE = 30;
  }

  // Command to move to specified position on field
  public static final class MoveCommand {
    // Pose2d containing acceptable target-difference error values
    public static final Pose2d ERROR = new Pose2d(0.4, 0.4, Rotation2d.fromDegrees(20));


    // FORWARD_* and HORIZONTAL* will probably be merged into a single set of DRIVE_* values
    public static final double DRIVE_P = 0.0;
    public static final double DRIVE_I = 0.0;
    public static final double DRIVE_D = 0.0;

    public static final double ROTATION_P = 0.0;
    public static final double ROTATION_I = 0.0;
    public static final double ROTATION_D = 0.0;
  }


  public static final class Camera {
    public static final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(16);
    public static final double CAMERA_PITCH_RADS = Units.degreesToRadians(45);
    public static final AprilTagFieldLayout field = null;  // FIXME: add json file

    // Position of camera relative to the robot
    public static final Transform3d cameraToRobot = new Transform3d();

//    static {
//      try {
//        field = new AprilTagFieldLayout("2024-crescendo.json");  // FIXME: Add JSON to deploy
//      } catch (IOException e) {
//        throw new RuntimeException(e);
//      }
//    }
  }

  // Electric current limits for intake motors
  public static final int FRONT_INTAKE_CURRENT_LIMIT = 15;
  public static final int BACK_INTAKE_CURRENT_LIMIT = 25;
  public static final double FRONT_INTAKE_NOTE_VOLTS = 6.0;
  public static final double BACK_INTAKE_NOTE_VOLTS = 8.0;



  public static class Launcher {
    public static final double ARM_CONVERSION_FACTOR = 80.0 / 9.0 * 58.0 / 11.0;

    public static final double POSITION_ERROR = 0.2;
    public static final double VELOCITY_ERROR = 0.2;
    public static final double SHOOTER_RADIUS = 1.063;
    public static final double TERMINAL_VELOCITY_ACCOUNTING = 0.1; //accounts for terminal velocity, probably no real need TODO: fine tune through testing

    // Heights for aimbot
    // TODO: fine tune speaker height for maximum variability resistance
    public static final double SPEAKER_HEIGHT = 6.8;
    public static final double JOINT_HEIGHT = 1.72283;

    // The desired upward velocity of the note when it enters the speaker
    // Higher values mean earlier entry and more upward, lower values mean more horizontal.
    // Values close to 0 are likely to hit the front instead of going in.
    public static final double ENTERING_SPEAKER_VELOCITY = 1.0;  // TODO: fine tune velocity through testing
    public static final double ACCELERATION_GRAVITY = -32.17405;  // Acceleration of gravity ft/s^2


    public static class Positions {
      // Won't actually be 0.0, origin will be when the arm is straight down
      public static final double INTAKE = 0.0;  // FIXME: unknown units & value
      public static final double AMP = 0.0;  // FIXME: unknown units & value


      // Values for testing arm movement. Set positions will be replaced with photonvision aimbot.
      public static final double UNDERHAND_SHOOT = 0.0;  // FIXME: unknown units & value
      public static final double OVERHAND_SHOOT = 0.0;  // FIXME: unknown units & value
    }

    public static class Velocities {
      public static final double GRAB = 0.0;  // FIXME: unknown units & value
      public static final double SHOOT = 0.0;  // FIXME: unknown units & value
    }
  }

  // Side of the field per-match
  public static Optional<DriverStation.Alliance> alliance = Optional.empty();

  private static Pose2d pose(double x, double y, double theta) {
    return new Pose2d(x, y, Rotation2d.fromDegrees(theta));
  }

  public static Pose2d allianceBasedPosition(Pose2d position) {
    return (Constants.alliance.isPresent() && Constants.alliance.get() == DriverStation.Alliance.Red)
      ? pose(FIELD_LENGTH - position.getX(), position.getY(), position.getRotation().getDegrees() + 180)
      : position;
  }

  public static final SendableChooser<Integer> positionChooser = new SendableChooser<>();
  public static final HashMap<String, Pose2d> STARTING_POSITIONS = new HashMap<>(Map.ofEntries(
    // Blue Alliance
    Map.entry("Amp Corner",                 pose(1.4997,  7.401295403, 0)),
    Map.entry("Amp-Side Speaker Corner",    pose(1.2813,  6.445974092, 0)),
    Map.entry("Speaker Center",             pose(1.2813,  5.556972314, 0)),
    Map.entry("Inward-Side Speaker Corner", pose(1.2813,  4.667970536, 0)),
    Map.entry("Field Center",               pose(1.4997,  4.11480823,  0)),
    Map.entry("Speaker-Side Bar Line",      pose(1.4997,  3.309042418, 0)),
    Map.entry("Opponent-Side Bar Line",     pose(1.4997,  2.572440945, 0))
  ));

  public static Pose2d STARTING_POSITION = new Pose2d(0.0, 0.0, new Rotation2d(0.0));

  public static void UpdateSettings() {
    Constants.alliance = DriverStation.getAlliance();

    Integer position = Constants.positionChooser.getSelected();
    if (position != null) {
      String positionString = Constants.STARTING_POSITIONS.keySet().toArray()[position].toString();
      if (!Constants.STARTING_POSITIONS.containsKey(positionString)) {
        Constants.STARTING_POSITION = Constants.STARTING_POSITIONS.get(positionString);
      } else {
        System.out.println("[ WARN ] The starting position was not updated properly: '" + positionString + "'");
      }
      SmartDashboard.putString("Auto Starting Position", positionString);
    }


    SmartDashboard.putString("Alliance", Constants.alliance.isPresent() ? Constants.alliance.get().toString() : "None");
    SmartDashboard.putString("Auto Starting Coords", Constants.STARTING_POSITION.toString());
  }

}
