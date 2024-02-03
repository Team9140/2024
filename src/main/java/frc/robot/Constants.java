// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

import java.util.Optional;

public final class Constants {
  public static final double LOOP_INTERVAL = 0.010;
  public static final double TRACK_WIDTH = Units.inchesToMeters(23.75);
  public static final double WHEEL_BASE = Units.inchesToMeters(20.75);
  public static final int WIDTH = 29;  // Inches
  public static final int LENGTH = 29;  // Inches

  public static final class Drivetrain {
    public static final double WHEEL_DIAMETER = 4;  // Diameter of each wheel
    public static final double DRIVE_DEADBAND = 0.10;  // Input deadband value for left joystick
    public static final double TURN_DEADBAND = 0.15;  // Input deadband value for right joystick

    // Max linear and rotational speeds
    public static final double METERS_PER_SECOND = Units.feetToMeters(16);
    public static final double ACCELERATION = Units.feetToMeters(48);
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
    public static final double DRIVE_P = 0.1;
    public static final double DRIVE_I = 0.0;
    public static final double DRIVE_D = 0.0;


    // Electric current limits for the swerve modules
    public static final int DRIVE_CURRENT_LIMIT = 25;
    public static final int TURN_CURRENT_LIMIT = 25;


    // PID wrapping values for the turn motor
    public static final double PID_MIN_INPUT = 0.0;
    public static final double PID_MAX_INPUT = 2 * Math.PI;
  }

  public static class Ports {
    public static final int INPUT_CONTROLLER = 0;  // Xbox Controller

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

  public static final double EPSILON = 0.00000001;
  public static Optional<DriverStation.Alliance> alliance = Optional.empty();

  public static void UpdateSettings() {
    Constants.alliance = DriverStation.getAlliance();
  }
}
