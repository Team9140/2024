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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
    public static final double WHEEL_DIAMETER = 4;  // Diameter of each wheel
    public static final double DRIVE_DEADBAND = 0.10;  // Input deadband value for left joystick
    public static final double TURN_DEADBAND = 0.15;  // Input deadband value for right joystick

    // Max linear and rotational speeds
    public static final double METERS_PER_SECOND = Units.feetToMeters(16);  // ft/s max velocity
    public static final double ACCELERATION = Units.feetToMeters(48);  // ft/s^2 max acceleration
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
    public static final double TURN_P = 0.4;
    public static final double TURN_I = 0.0;
    public static final double TURN_D = 0.0;


    // PID values for the drive motor
//    public static final double DRIVE_P = 0.1;
//    public static final double DRIVE_I = 0.0;
//    public static final double DRIVE_D = 0.0;


    // Electric current limits for the swerve modules
    public static final int DRIVE_CURRENT_LIMIT = 25;
    public static final int TURN_CURRENT_LIMIT = 25;


    // PID wrapping values for the turn motor
    public static final double PID_MIN_INPUT = 0.0;
    public static final double PID_MAX_INPUT = 2 * Math.PI;

    // Distance to travel before rotation is attempted
//    public static final double ROTATION_DELAY_METERS = 1.0;
  }

  public static class Ports {
    public static final int INPUT_CONTROLLER = 0;  // Xbox Controller
    public static final String CAMERA = "ProblemCamera";  // PhotonVision camera ID

    // Ports for CANSparkMax motor controllers
    public static final int FRONT_LEFT_DRIVE = 10;
    public static final int FRONT_LEFT_TURN = 5;
    public static final int FRONT_RIGHT_DRIVE = 3;
    public static final int FRONT_RIGHT_TURN = 4;
    public static final int BACK_LEFT_DRIVE = 6;
    public static final int BACK_LEFT_TURN = 7;
    public static final int BACK_RIGHT_DRIVE = 2;
    public static final int BACK_RIGHT_TURN = 1;

    public static final String LAUNCHER_CANBUS = "Jama";
    public static final int ARM_MOTOR = 5;
    public static final int BOTTOM_SHOOTER = 6;
    public static final int TOP_SHOOTER = 7;
    public static final int ARM_FEEDER = 11;

    public static final int FRONT_INTAKE = 0;
    public static final int BACK_INTAKE = 0;
  }

  // Command to move to specified position on field
  public static final class MoveCommand {
    // Pose2d containing acceptable target-difference error values
    public static final Pose2d ERROR = new Pose2d(0.4, 0.4, Rotation2d.fromDegrees(20));


    // FORWARD_* and HORIZONTAL* will probably be merged into DRIVE_*
    public static final double FORWARD_P = 0.0;
    public static final double FORWARD_I = 0.0;
    public static final double FORWARD_D = 0.0;

    public static final double HORIZONTAL_P = 0.0;
    public static final double HORIZONTAL_I = 0.0;
    public static final double HORIZONTAL_D = 0.0;

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
//        field = new AprilTagFieldLayout("2024-crescendo.json");
//      } catch (IOException e) {
//        throw new RuntimeException(e);
//      }
//    }
  }

  public static class Launcher {
    public static final double ARM_CONVERSION_FACTOR = 0.0;  // FIXME: unknown value

    // Electric current limits for intake motors
    public static final int INTAKE_CURRENT_LIMIT = 25;
    public static final double INTAKE_NOTE_VOLTS = 8.0;

    public static final double POSITION_ERROR = 0.2;
    public static final double VELOCITY_ERROR = 0.2;

    public static class Positions {
      public static final double INTAKE = 0.0;
//      public static final double
    }
    public static class Velocities {
      public static final double GRAB = 0.0;  // FIXME: unknown units & values
      public static final double SHOOT = 0.0;  // FIXME: unknown units & values
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

    SmartDashboard.putString("Alliance", Constants.alliance.get().toString());
    SmartDashboard.putNumber("Position", Constants.alliance_position.getAsInt());
  }

}
