// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import java.io.IOException;
import java.util.Optional;

public final class Constants {
  public static final double LOOP_INTERVAL = 0.020;
  public static final double wheelBase = Units.inchesToMeters(0);
  public static final double trackWidth = Units.inchesToMeters(0);

  //  public static final double fieldx = Units.inchesToMeters(501);
  //  public static final double fieldy = Units.inchesToMeters(323.28);
  public static final int kencoderCountsPerRev = 1;
  public static final class Drivetrain {
    public static final double positionConversionFactor = 2 * Math.PI;
    public static final double driveWheelDiameter = 4;
    public static final double DEADBAND = 1.0;

    public static final double FORWARD_METERS_PER_SECOND = Units.feetToMeters(16);
    public static final double HORIZONTAL_METERS_PER_SECOND = Units.feetToMeters(16);
    public static final double ROTATION_RADIANS_PER_SECOND = Math.PI;
  }

  public static class Ports {
    public static final int INPUT_CONTROLLER = 0;
    public static final int GYRO = 0;
    public static final String CAMERA = "ProblemCamera";
    public static final int frontLeftDrivePort = 1;
    public static final int frontLeftTurnPort = 2;
    public static final int frontRightDrivePort = 3;
    public static final int frontRightTurnPort = 4;
    public static final int backLeftDrivePort = 5;
    public static final int backLeftTurnPort = 6;
    public static final int backRightDrivePort = 7;
    public static final int backRightTurnPort = 8;
  }


  public static final class Camera {
    public static final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(16);
    public static final double CAMERA_PITCH_RADS = Units.degreesToRadians(45);
    public static final AprilTagFieldLayout field;

    static {
      try {
        field = new AprilTagFieldLayout("2024-crescendo.json");
      } catch (IOException e) {
        throw new RuntimeException(e);
      }
    }

    public static final Transform2d cameraToRobot = new Transform2d(12, 12, new Rotation2d(0));
  }

  public static Optional<DriverStation.Alliance> alliance = Optional.empty();

  public static void UpdateSettings() {
    Constants.alliance = DriverStation.getAlliance();
  }
}
