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

import java.util.Optional;
import java.util.OptionalInt;

public final class Constants {
  public static final double LOOP_INTERVAL = 0.010;  // Periodic interval delay time FIXME: unknown units
  public static final double TRACK_WIDTH = Units.inchesToMeters(23.75);  // Horizontal (side-to-side) distance between wheels
  public static final double WHEEL_BASE = Units.inchesToMeters(20.75);  // Front-to-back distance between wheels
  public static final double scoringRange = 120.0;  // FIXME: Unknown

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
//    public static final double DRIVE_P = 0.1;
//    public static final double DRIVE_I = 0.0;
//    public static final double DRIVE_D = 0.0;


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

    public static final int CLIMBER = 0;

    public static final int FRONT_LEFT_INTAKE = 33;
    public static final int FRONT_RIGHT_INTAKE = 30;
    public static final int BACK_INTAKE = 31;
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
  }

  public static final int CANDLE_LEDS_PER_ANIMATION = 30;  // FIXME: Placeholder value

  public static final double CANDLE_DURATION = 4;
  public static final double CANDLE_BRIGHTNESS = 1.0;

  // Electric current limits for intake motors
  public static final int FRONT_INTAKE_CURRENT_LIMIT = 15;
  public static final int BACK_INTAKE_CURRENT_LIMIT = 25;
  public static final double FRONT_INTAKE_NOTE_VOLTS = 6.0;
  public static final double BACK_INTAKE_NOTE_VOLTS = 8.0;
  public static final double AUTO_SPEED = 5; //FIXME: Add real values

  public static class Arm {
    // PID and SVA, used in motion magic
    public static final double P = 26.0;
    public static final double I = 0.0;
    public static final double D = 1.54;
    public static final double S = 0.14178;
    public static final double V = 0.94316;
    public static final double A = 0.07;
    public static final double MAX_CURRENT = 40.0; // Amps
    public static final double SENSOR_TO_MECHANISM_RATIO = 80.0 / 9.0 * 58.0 / 11.0 / (2 * Math.PI); // Radian rotations of arm

    // Motion Magic Specific Limits
    public static final double CRUISE_VELOCITY = 12.0; // Radians per second
    public static final double ACCELERATION = 24.0; // Radians per second per second
    public static final double FEED_FORWARD = 0.0; // FIXME: for later
    public static final double INITIAL_VARIANCE = Units.degreesToRadians(3); // Radians
    public static final double AIM_ERROR = 0.0; // FIXME: ask gijspice

    // Positions in radians
    public static class Positions {
      public static final double INTAKE = -1.69;
      public static final double AMP = 2.0;
      public static final double UNDERHAND = -0.3 * Math.PI;
      public static final double OVERHAND = 0.25 * Math.PI;
    }
  }

  public static class Thrower {
    public static class Launcher {
      public static final double MAX_CURRENT = 30.0; // amps
      public static final double INTAKE_VOLTAGE = -3.0; // volts
      public static final double SPEAKER_VOLTAGE = 8.0; // volts
      public static final double AMP_VOLTAGE = 3.0; // volts
    }

    public static class Feeder {
      public static final int MAX_CURRENT = 0;  // amps // FIXME: ACTUALLY PUT A VALUE
      public static final double INTAKE_VOLTAGE = -4.0; // volts
      public static final double PREPARE_VOLTAGE = -1.5; // volts
      public static final double LAUNCH_VOLTAGE = 12.0; // volts
    }
  }

  // Side of the field per-match
  public static Optional<DriverStation.Alliance> alliance = Optional.empty();
  public static OptionalInt alliance_position = OptionalInt.empty();

  public static final Pose2d[] STARTING_POSITIONS = {
    new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0)),  // Unknown
    new Pose2d(613.1875, 47.3541, Rotation2d.fromDegrees(180)),  // Red1
    new Pose2d(613.1875, 142.0625, Rotation2d.fromDegrees(180)),  // Red2
    new Pose2d(613.1875, 236.7708, Rotation2d.fromDegrees(180)),  // Red3
    new Pose2d(38.0625, 47.3541, Rotation2d.fromDegrees(0)),  // Blue1
    new Pose2d(38.0625, 142.0625, Rotation2d.fromDegrees(0)),  // Blue2
    new Pose2d(38.0625, 236.7708, Rotation2d.fromDegrees(0))  // Blue3
  };

  public static void UpdateSettings() {
    Constants.alliance = DriverStation.getAlliance();
    Constants.alliance_position = DriverStation.getLocation();
  }

}
