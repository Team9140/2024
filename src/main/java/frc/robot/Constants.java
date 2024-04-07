// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import lib.util.Util;

import java.util.HashMap;
import java.util.Map;

/**
  * A class containing settings and values needed by several subsystems
 **/
public final class Constants {
  public enum SYSID {
    Teleop,  // Normal driving mode
    RotationSysId,  // Full-body rotation SysId test
    StraightSysId,  // Full-body drive SysId test
    ThrowerRollerSysId  // Thrower roller SysId test TODO: move from the other repo
  };

  // A toggle for setting the teleop mode
  public static final Constants.SYSID SYSID_MODE = SYSID.Teleop;

  // Periodic interval delay time in seconds
  public static final double LOOP_INTERVAL = 0.01;

  // The feeder current minimum to buzz the driver's controller when reached
  public static final double INTAKE_NOTIFY_CURRENT = 8.0;

  // Size of the field
  //  public static final double fieldx = Units.inchesToMeters(501);
  //  public static final double fieldy = Units.inchesToMeters(323.28);

  public static final double CAMERA_RANGE = 10.0;
  public static final double SCORING_RANGE = 120.0;

  public static final class Drivetrain {
    // The offsets for swerve kinematics
    public static final double FRONT_OFFSET = Units.inchesToMeters(8.625);
    public static final double BACK_OFFSET = Units.inchesToMeters(-11.125);
    public static final double LEFT_OFFSET = Units.inchesToMeters(11.875);
    public static final double RIGHT_OFFSET = Units.inchesToMeters(-11.875);

    // The gear ratio for the drive motors
    public static final double DRIVE_GEAR_RATIO = (50.0 / 16.0) * (16.0 / 28.0) * (45.0 / 15.0);
    // Diameter of drive wheels in inches
    public static final double WHEEL_DIAMETER = 4.0;

    // Input deadband values for the joysticks
    public static final double DRIVE_DEADBAND = 0.10;
    public static final double TURN_DEADBAND = 0.15;

    // PID values for the right joystick
    public static final double TURN_JOY_P = 0.08;  // Was 0.1
    public static final double TURN_JOY_I = 0.0;
    public static final double TURN_JOY_D = 0.0;

    // Max linear velocity is m/s
    public static final double LINEAR_VELOCITY = Units.feetToMeters(19);
    // Max acceleration in m/s^2
    public static final double ACCELERATION = Units.feetToMeters(80);
    // Max rotational velocity in rad/s
    public static final double ROTATION_VELOCITY = (2 * Math.PI) * 2;  // 2 complete rotations per second (720 deg/s)

    // PID values for the turn motor controllers
    // https://www.chiefdelphi.com/t/finally-i-understand-pid/450811
    public static final double TURN_P = 1.97;
    public static final double TURN_I = 0.0;
    public static final double TURN_D = 0.115;


    // PID values for the drive motor controllers
    // https://www.chiefdelphi.com/t/finally-i-understand-pid/450811
    public static final double DRIVE_P = 4.0;
    public static final double DRIVE_I = 0.0;
    public static final double DRIVE_D = 0.0;


    // Electric current limits for the drive motors
    public static final int DRIVE_CURRENT_LIMIT = 60;

    // Lower bound wrapping value for the turn motor encoder
    public static final double PID_MIN_INPUT = 0.0;
    // Upper bound wrapping value for the turn motor encoder
    public static final double PID_MAX_INPUT = 2 * Math.PI;

    // Speed multipliers for different states of driving
    public static class Limits {
      public static double NORMAL_LIN = 1.0;
      public static double NORMAL_ROT = 1.0;

      public static double INTAKE_LIN = 0.8;
      public static double INTAKE_ROT = 0.6;

      public static double AMP_LIN = 0.6;
      public static double AMP_ROT = 0.4;

      public static double OVERHAND_LIN = 0.9;
      public static double OVERHAND_ROT = 0.7;

      public static double UNDERHAND_LIN = 0.9;
      public static double UNDERHAND_ROT = 0.7;
    }

    // Feedforward values for individual swerve modules
    // https://www.chiefdelphi.com/uploads/default/original/3X/d/8/d8c45602d966d596cd37b43017385db0b5a36bc7.pdf
    public static class FF {
      // Front left
      public static double FLS = 0.12844;
      public static double FLV = 2.215 * 1.0141;
      public static double FLA = 0.27452;

      // Front right
      public static double FRS = 0.12844;
      public static double FRV = 2.185 * 0.96;
      public static double FRA = 0.27452 * 0.5;

      // Back left
      public static double BLS = 0.12844;
      public static double BLV = 2.136 * 1.0567;
      public static double BLA = 0.27452;

      // Back right
      public static double BRS = 0.12844;
      public static double BRV = 2.09 * 0.96 * 1.0226;
      public static double BRA = 0.27452 * 0.5;
    }
  }

  /**
    * Individual device ids
   **/
  public static class Ports {
    public static final int DRIVER_CONTROLLER = 0;  // Xbox Controller
    public static final int AUXILIARY_CONTROLLER = 1;

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

    public static final String CAMERA = "limelight";
  }

  public static final int CANDLE_LEDS_PER_ANIMATION = 30;  // FIXME: Placeholder value
  public static final double CANDLE_BRIGHTNESS = 1.0;

  // Electric current limits for intake motors
  public static final int FRONT_INTAKE_CURRENT_LIMIT = 15;
  public static final int BACK_INTAKE_CURRENT_LIMIT = 15;
  public static final double FRONT_INTAKE_NOTE_VOLTS = 9.0;
  public static final double BACK_INTAKE_NOTE_VOLTS = 7.0;

  public static class Arm {
    // PID and SVA, used in motion magic
    public static final double P = 26.0;
    public static final double I = 0.0;
    public static final double D = 1.54;
    public static final double S = 0.14178;
    public static final double V = 0.94316;
    public static final double A = 0.07;
    public static final double MAX_CURRENT = 48.0;  // Amps
    public static final double SENSOR_TO_MECHANISM_RATIO = 80.0 / 9.0 * 58.0 / 11.0 / (2 * Math.PI);  // Radian rotations of arm

    // Motion Magic Specific Limits
    public static final double CRUISE_VELOCITY = 24.0;  // Radians per second
    public static final double ACCELERATION = 36.0;  // Radians per second per secon
    public static final double FEED_FORWARD = 0.0;  // FIXME: for later
    public static final double INITIAL_VARIANCE = Units.degreesToRadians(5);  // Radians
    public static final double AIM_ERROR = Math.toRadians(1.0);

    // Positions in radians
    public static class Positions {
      public static final double INTAKE = -1.7;
      public static final double AMP = 2.00;

      public static final double UNDERHAND = -Math.PI / 3.5;
      public static final double OVERHAND = 0.25 * Math.PI;
    }
  }

  public static class Thrower {
    public static class Launcher {
      public static final double MAX_CURRENT = 60.0;  // amps
      public static final double INTAKE_VOLTAGE = -6.0;  // volts
      public static final double SPEAKER_VOLTAGE = 12.0;  // volts
      public static final double TOP_AMP_VOLTAGE = 4.0;  // volts
      public static final double BOTTOM_AMP_VOLTAGE = 2.0;  // volts
    }

    public static class Feeder {
      public static final int MAX_CURRENT = 20;  // amps FIXME: ACTUALLY PUT A VALUE
      public static final double INTAKE_VOLTAGE = -4.0;  // volts
      public static final double PREPARE_VOLTAGE = -1.5;  // volts
      public static final double LAUNCH_VOLTAGE = 12.0;  // volts
    }
  }

  public static class Climber {
    public static final double UP_VELOCITY = 1.0;
    public static final double ERROR = 1.0;
    public static final double UP_POSITION = 113.0;  // FIXME
    public static final double DOWN_POSITION = 240.0;  // FIXME
  }

  public static final class Camera {
    public static final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(16);
    public static final double CAMERA_PITCH_RADS = Units.degreesToRadians(45);
    public static final AprilTagFieldLayout field = null;  // FIXME: add json file

    // Position of camera relative to the robot
    public static final Transform3d cameraToRobot = new Transform3d();
  }

  public static class Auto {
    public static final int DEFAULT_POSITION = 0;
    public static final HashMap<String, Pose2d> POSITIONS = new HashMap<>(Map.ofEntries(
      // Blue Alliance
      Map.entry("Amp Side", Util.pose(0.71,  6.69,  60.00)),
      Map.entry("Mid Side", Util.pose(1.369, 5.552, 0.0)),
      Map.entry("Ref Side", Util.pose(0.71,  4.37,  -60.00))
    ));

    public static final int DEFAULT_MODE = 1;  // Set 4-note auto as default
    public static final int DISABLED_ID = 999;
    public static final String[] REGULAR = {
      "Shoot & drive",
      "Middle 4-Note",
      "Preloaded stationary"
    };

    public static final int CHOREO_OFFSET = 100;
    public static final HashMap<String, String> CHOREO = new HashMap<>(Map.ofEntries(
      Map.entry("Test", "testStraightAuto")
    ));

    public static final int PATHPLANNER_OFFSET = 200;
    public static final HashMap<String, String> PATHPLANNER = new HashMap<>(Map.ofEntries(
      // Add more here
    ));

    public static final Pose2d WAIT_UNTIL_POSITION_ERROR = Util.pose(0.25, 0.25, 20.0);

    // The radius of the robot, for use with PathPlanner
    public static final int BASE_RADIUS = 15;
  }
}
