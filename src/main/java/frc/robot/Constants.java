// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

import java.io.IOException;
import java.util.Optional;

public final class Constants {
  public static final double LOOP_INTERVAL = 0.010;  // Periodic interval delay time FIXME: unknown units
  public static final double TRACK_WIDTH = Units.inchesToMeters(23.75);  // Horizontal (side-to-side) distance between wheels
  public static final double WHEEL_BASE = Units.inchesToMeters(20.75);  // Front-to-back distance between wheels
  public static final double scoringRange = 120.0;  // FIXME: Unknown

  // Full-body dimensions
  public static final int WIDTH = 29;  // Inches, front-to-back width
  public static final int LENGTH = 29;  // Inches, side-to-side length

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

//    // Distance to travel before rotation is attempted
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
    public static final AprilTagFieldLayout field;

    // Position of camera relative to the robot
    public static final Transform3d cameraToRobot = new Transform3d();

    static {
      try {
        field = new AprilTagFieldLayout("2024-crescendo.json");
      } catch (IOException e) {
        throw new RuntimeException(e);
      }
    }
  }

  // Side of the field per-match
  public static Optional<DriverStation.Alliance> alliance = Optional.empty();

  public static void UpdateSettings() {
    Constants.alliance = DriverStation.getAlliance();
  }
}
