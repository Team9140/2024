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

/**
  * A class containing settings and values needed by several subsystems
 **/
public final class Constants {
  /**
    * Periodic interval delay time in seconds
   **/
  public static final double LOOP_INTERVAL = 0.010;
  /**
    * The scoring range for PhotonVision
    * FIXME: Unknown purpose
   **/
  public static final double scoringRange = 120.0;

  /**
    * The offset for the front motors. For swerve kinematics.
   **/
  public static final double FRONT_OFFSET = Units.inchesToMeters(8.625);
  /**
    * The offset for the back motors. For swerve kinematics.
   **/
  public static final double BACK_OFFSET = Units.inchesToMeters(-11.125);
  /**
    * The offset for the left motors. For swerve kinematics.
   **/
  public static final double LEFT_OFFSET = Units.inchesToMeters(11.875);
  /**
    * The offset for the right motors. For swerve kinematics.
   **/
  public static final double RIGHT_OFFSET = Units.inchesToMeters(-11.875);

  /**
    * The radius of the robot for usage by PathPlanner
   **/
  public static final int BASE_RADIUS = 15;

  // Size of the field
  //  public static final double fieldx = Units.inchesToMeters(501);
  //  public static final double fieldy = Units.inchesToMeters(323.28);
  public static final class Drivetrain {
    /**
      * The gear ratio for the drive motors
     **/
    public static final double DRIVE_GEAR_RATIO = (50.0 / 16.0) * (16.0 / 28.0) * (45.0 / 15.0);
    /**
      * Diameter of each wheel in inches
     **/
    public static final double WHEEL_DIAMETER = 4;
    /**
      * Input deadband value for left joystick
     **/
    public static final double DRIVE_DEADBAND = 0.10;
    /**
      * Input deadband value for right joystick
     **/
    public static final double TURN_DEADBAND = 0.15;

    /**
      * Max linear velocity in m/s
      */
    public static final double LINEAR_VELOCITY = Units.feetToMeters(19);
    /**
      * Max acceleration in m/s^2
     **/
    public static final double ACCELERATION = Units.feetToMeters(80);  // ft/s^2 max acceleration
    /**
      * Max rotational velocity in rad/s
     **/
    public static final double ROTATION_VELOCITY = (2 * Math.PI) * 2;  // 2 complete rotations per second (720 deg/s)


    /**
      * The kS value. This is part of the drive motor controller's feedforward.
      * @see <a href="https://www.chiefdelphi.com/uploads/default/original/3X/d/8/d8c45602d966d596cd37b43017385db0b5a36bc7.pdf">Understanding Feedforward Models for FRC</a>
     **/
    public static final double MODULE_S = 0.12844;
    /**
      * The motor speed constant. This is part of the drive motor controller's feedforward.
      * @see <a href="https://www.chiefdelphi.com/uploads/default/original/3X/d/8/d8c45602d966d596cd37b43017385db0b5a36bc7.pdf">Understanding Feedforward Models for FRC</a>
     **/
    public static final double MODULE_V = 2.1253;
    /**
      * The kA value. This is part of the drive motor controller's feedforward.
      * @see <a href="https://www.chiefdelphi.com/uploads/default/original/3X/d/8/d8c45602d966d596cd37b43017385db0b5a36bc7.pdf">Understanding Feedforward Models for FRC</a>
     **/
    public static final double MODULE_A = 0.27452;


    /**
      * PID value for the turn motor controller
      * @see <a href="https://www.chiefdelphi.com/t/finally-i-understand-pid/450811">Finally I Understand PID! - chiefdelphi</a>
     **/
    public static final double TURN_P = 3.8069;
    /**
      * PID value for the turn motor controller
      * @see <a href="https://www.chiefdelphi.com/t/finally-i-understand-pid/450811">Finally I Understand PID! - chiefdelphi</a>
     **/
    public static final double TURN_I = 0.0;
    /**
      * PID value for the turn motor controller
      * @see <a href="https://www.chiefdelphi.com/t/finally-i-understand-pid/450811">Finally I Understand PID! - chiefdelphi</a>
     **/
    public static final double TURN_D = 0.15584;


    /**
      * PID value for the drive motor controller
      * @see <a href="https://www.chiefdelphi.com/t/finally-i-understand-pid/450811">Finally I Understand PID! - chiefdelphi</a>
     **/
    public static final double DRIVE_P = 6.366;
    /**
      * PID value for the drive motor controller
      * @see <a href="https://www.chiefdelphi.com/t/finally-i-understand-pid/450811">Finally I Understand PID! - chiefdelphi</a>
     **/
    public static final double DRIVE_I = 0.0;
    /**
      * PID value for the drive motor controller
      * @see <a href="https://www.chiefdelphi.com/t/finally-i-understand-pid/450811">Finally I Understand PID! - chiefdelphi</a>
     **/
    public static final double DRIVE_D = 0.0;


    /**
      * Electric current limits for the drive motors
     **/
    public static final int DRIVE_CURRENT_LIMIT = 60;
//    public static final int TURN_CURRENT_LIMIT = 30;


    /**
      * Lower bound wrapping value for the turn motor encoder
     **/
    public static final double PID_MIN_INPUT = 0.0;
    /**
     * Upper bound wrapping value for the turn motor encoder
     **/
    public static final double PID_MAX_INPUT = 2 * Math.PI;

    // Distance to travel before rotation is attempted
//    public static final double ROTATION_DELAY_METERS = 1.0;
  }

  /**
    * All of the device Ids
   **/
  public static class Ports {
    public static final int INPUT_CONTROLLER = 0;  // Xbox Controller
    public static final String CAMERA = "ProblemCamera";  // PhotonVision camera ID

    public static final int CANDLE = 5;

    // Ports for CANSparkMax motor controllers
    public static final int FRONT_LEFT_DRIVE = 1;
    public static final int FRONT_LEFT_TURN = 4;
    public static final int FRONT_RIGHT_DRIVE = 0;
    public static final int FRONT_RIGHT_TURN = 9;
    public static final int BACK_LEFT_DRIVE = 3;
    public static final int BACK_LEFT_TURN = 7;
    public static final int BACK_RIGHT_DRIVE = 2;
    public static final int BACK_RIGHT_TURN = 1;

    public static final String CTRE_CANBUS = "jama";
    public static final int ARM_MOTOR = 4;
    public static final int BOTTOM_LAUNCHER = 5;
    public static final int TOP_LAUNCHER = 6;
    public static final int THROWER_FEEDER = 32;

    public static final int CLIMBER = 6;

    public static final int FRONT_LEFT_INTAKE = 33;
    public static final int FRONT_RIGHT_INTAKE = 30;
    public static final int BACK_INTAKE = 31;
  }

  /**
    * Values for usage by the vision camera
   **/
  public static final class Camera {
    public static final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(16);
    public static final double CAMERA_PITCH_RADS = Units.degreesToRadians(45);
    public static final AprilTagFieldLayout field = null;  // FIXME: add json file

    // Position of camera relative to the robot
    public static final Transform3d cameraToRobot = new Transform3d();
  }

  public static final int CANDLE_LEDS_PER_ANIMATION = 30;  // FIXME: Placeholder value

//  public static final double CANDLE_DURATION = 4;
  public static final double CANDLE_BRIGHTNESS = 1.0;

  // Electric current limits for intake motors
  public static final int FRONT_INTAKE_CURRENT_LIMIT = 15;
  public static final int BACK_INTAKE_CURRENT_LIMIT = 25;
  public static final double FRONT_INTAKE_NOTE_VOLTS = 6.0;
  public static final double BACK_INTAKE_NOTE_VOLTS = 8.0;
  public static final double AUTO_SPEED = 5; // FIXME: Add real values

  public static class Arm {
    // PID and SVA, used in motion magic
    public static final double P = 26.0;
    public static final double I = 0.0;
    public static final double D = 1.54;
    public static final double S = 0.14178;
    public static final double V = 0.94316;
    public static final double A = 0.07;
    public static final double MAX_CURRENT = 40.0;  // Amps
    public static final double SENSOR_TO_MECHANISM_RATIO = 80.0 / 9.0 * 58.0 / 11.0 / (2 * Math.PI);  // Radian rotations of arm

    // Motion Magic Specific Limits
    public static final double CRUISE_VELOCITY = 12.0;  // Radians per second
    public static final double ACCELERATION = 24.0;  // Radians per second per second
    public static final double FEED_FORWARD = 0.0;  // FIXME: for later
    public static final double INITIAL_VARIANCE = Units.degreesToRadians(3);  // Radians
    public static final double AIM_ERROR = Math.toRadians(3.0);  // FIXME: ask gijspice

    // Positions in radians
    public static class Positions {
      public static final double INTAKE = -1.69;
      public static final double AMP = 1.95;

      public static final double UNDERHAND = -1.4;
      public static final double OVERHAND = 0.25 * Math.PI;
    }
  }

  public static class Thrower {
    public static class Launcher {
      public static final double MAX_CURRENT = 30.0;  // amps
      public static final double INTAKE_VOLTAGE = -3.0;  // volts
      public static final double SPEAKER_VOLTAGE = 11.0;  // volts
      public static final double TOP_AMP_VOLTAGE = 7.0;  // volts
      public static final double BOTTOM_AMP_VOLTAGE = 3.0;  // volts
      public static final double LAUNCH_SPEED = 1669.756495; //ft/s
    }

    public static class Feeder {
      public static final int MAX_CURRENT = 0;  // amps FIXME: ACTUALLY PUT A VALUE
      public static final double INTAKE_VOLTAGE = -4.0;  // volts
      public static final double PREPARE_VOLTAGE = -1.5;  // volts
      public static final double LAUNCH_VOLTAGE = 12.0;  // volts
    }

    public static class AutoAim {
      public static final double ACCELERATION_GRAVITY = 32.17405; // acceleration of gravity, ft/s
      public static final double ENTERING_SPEAKER_VELOCITY = 1.0; // ft/s value of the upwards velocity of the note when it enters the speaker
      public static final double SPEAKER_HEIGHT = 6.8; // the ft height of the speaker entrance, where the bot is aiming
      public static final double JOINT_HEIGHT = 1.72283; // how high the bot thinks it is shooting from
      public static final double ANGLE_ERROR_FIX = 0.1; //FIX ME if we using this need to fine tune it
    }
  }

  public static class Climber {
    public static final double UP_VELOCITY = 1.0;
    public static final double DOWN_VELOCITY = 1.0;
    public static final double ERROR = 1.0;
    public static final double UP_POSITION = 0.0;  // FIXME
    public static final double DOWN_POSITION = 0.0;  // FIXME
  }

  // Side of the field per-match
  public static Optional<DriverStation.Alliance> alliance = Optional.empty();

  private static Pose2d pose(double x, double y, double theta) {
    return new Pose2d(x, y, Rotation2d.fromDegrees(theta));
  }

  public static final SendableChooser<Integer> positionChooser = new SendableChooser<>();
  public static final HashMap<String, Pose2d> STARTING_POSITIONS = new HashMap<>(Map.ofEntries(
    // Blue Alliance
    Map.entry("Amp Side", pose(0.71, 6.69, 60.00)),
    Map.entry("Mid Side", pose(1.63, 5.54, 0)),
    Map.entry("Ref Side", pose(0.71, 4.37, -60.00))
//    Map.entry("Speaker Center",             pose(1.2813,  5.556972314, 0)),
//    Map.entry("Inward-Side Speaker Corner", pose(1.2813,  4.667970536, 0)),
//    Map.entry("Field Center",               pose(1.4997,  4.11480823,  0)),
//    Map.entry("Speaker-Side Bar Line",      pose(1.4997,  3.309042418, 0)),
//    Map.entry("Opponent-Side Bar Line",     pose(1.4997,  2.572440945, 0))
  ));

  public static final String[] REGULAR_AUTOS = {
    "Shoot & drive",
    "Amp-side 4-note",
    "2-note",
    "3-note"
  };
  public static final HashMap<String, String> CHOREO_AUTOS = new HashMap<>(Map.ofEntries(
    Map.entry("Abhinav's Test Auto", "auto1")
  ));
  public static final HashMap<String, String> PATHPLANNER_AUTOS = new HashMap<>(Map.ofEntries(
//      Map.entry("Blue Amp Side", "BlueAmpSideTriple"),  // NOT WORKING
//      Map.entry("Blue Mid Side", "BlueMidSideTriple"),  // NOT WORKING
//      Map.entry("Blue Ref Side", "BlueRefSideTriple"),  // NOT WORKING
//      Map.entry("Blue Leave Source", "LEAVE"),
//      Map.entry("Red Amp Side", "RedAmpSideTriple")
  ));

  public static final int CHOREO_AUTOS_OFFSET = 100;
  public static final int PATHPLANNER_AUTOS_OFFSET = 200;

  public static final int DEFAULT_STARTING_POSITION = 0;
  public static Pose2d STARTING_POSITION;

  public static final int DEFAULT_AUTO = 0;  // Set first "regular" auto as default

  public static void UpdateSettings() {
    Constants.alliance = DriverStation.getAlliance();
    Integer position = Constants.positionChooser.getSelected();
    if (position != null) {
      String positionString = Constants.STARTING_POSITIONS.keySet().toArray()[position].toString();

      if (Constants.STARTING_POSITIONS.containsKey(positionString)) {
        Pose2d startingPosition = Constants.STARTING_POSITIONS.get(positionString);
        Constants.STARTING_POSITION = Constants.alliance.isPresent() && Constants.alliance.get() == DriverStation.Alliance.Red
          ? new Pose2d(startingPosition.getX(), startingPosition.getY(), Rotation2d.fromDegrees(startingPosition.getRotation().getDegrees() * -1))
          : startingPosition;
      } else {
        System.out.println("[ WARN ] The starting position was not updated properly: '" + positionString + "'");
      }
      SmartDashboard.putString("Auto Starting Position", positionString);
    } else {
      // Set the starting position to the default starting position if it cannot read a value from SmartDashboard
      System.out.println("[ WARN ] The starting position was not updated properly");
      Constants.STARTING_POSITION = Constants.STARTING_POSITIONS.get(Constants.STARTING_POSITIONS.keySet().toArray()[Constants.DEFAULT_STARTING_POSITION]);
    }
    SmartDashboard.putString("Auto Starting Coords", Constants.STARTING_POSITION.toString());
    SmartDashboard.putString("Alliance", Constants.alliance.isPresent() ? Constants.alliance.get().toString() : "None");
  }
}
