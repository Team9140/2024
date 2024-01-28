// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.Optional;

public final class Constants {
  public static final double LOOP_INTERVAL = 0.020;
  public static final double trackWidth = Units.inchesToMeters(23.75);
  public static final double wheelBase = Units.inchesToMeters(20.75);

  public static final class Drivetrain {
    public static final double positionConversionFactor = 2 * Math.PI / (150 / 7);
    public static final double driveWheelDiameter = 4;
    public static final double DRIVE_DEADBAND = 0.10;
    public static final double TURN_DEADBAND = 0.10;

    public static final double FORWARD_METERS_PER_SECOND = Units.feetToMeters(16);
    public static final double HORIZONTAL_METERS_PER_SECOND = Units.feetToMeters(16);
    public static final double ROTATION_RADIANS_PER_SECOND = 4 * Math.PI;

    public static final double MODULE_S = 0;
    public static final double MODULE_V = 2.43219076;
    public static final double MODULE_A = 0;

    public static final double FRONT_LEFT_KENCODER_OFFSET = 0.605;
    public static final double FRONT_RIGHT_KENCODER_OFFSET = 0.795;
    public static final double BACK_LEFT_KENCODER_OFFSET = 0.320;
    public static final double BACK_RIGHT_KENCODER_OFFSET = 0.375;

    public static final double TURN_P = 0.4;
    public static final double TURN_I = 0.0;
    public static final double TURN_D = 0.0;

    public static final int DRIVE_CURRENT_LIMIT = 25;
    public static final int TURN_CURRENT_LIMIT = 25;

    public static final double PID_MIN_INPUT = 0.0;
    public static final double PID_MAX_INPUT = 2 * Math.PI;
  }

  public static class Ports {
    public static final int INPUT_CONTROLLER = 0;
    public static final int GYRO = 0;
    public static final int frontLeftDrivePort = 10;
    public static final int frontLeftTurnPort = 5;
    public static final int frontRightDrivePort = 3;
    public static final int frontRightTurnPort = 4;
    public static final int backLeftDrivePort = 6;
    public static final int backLeftTurnPort = 7;
    public static final int backRightDrivePort = 2;
    public static final int backRightTurnPort = 1;
  }

  public static Optional<DriverStation.Alliance> alliance = Optional.empty();

  public static void UpdateSettings() {
    Constants.alliance = DriverStation.getAlliance();
  }
}
