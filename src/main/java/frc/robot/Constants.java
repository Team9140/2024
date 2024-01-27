// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.Optional;

public final class Constants {
  public static final double LOOP_INTERVAL = 0.020;
  public static final double wheelBase = Units.inchesToMeters(0);
  public static final double trackWidth = Units.inchesToMeters(0);

  public static final class Drivetrain {
    public static final double positionConversionFactor = 2 * Math.PI;
    public static final double driveWheelDiameter = 4;
  }

  public static class Ports {
    public static final int INPUT_CONTROLLER = 0;
    public static final int GYRO = 0;
    public static final int frontLeftDrivePort = 1;
    public static final int frontLeftTurnPort = 2;
    public static final int frontRightDrivePort = 3;
    public static final int frontRightTurnPort = 4;
    public static final int backLeftDrivePort = 5;
    public static final int backLeftTurnPort = 6;
    public static final int backRightDrivePort = 7;
    public static final int backRightTurnPort = 8;
  }

  public static Optional<DriverStation.Alliance> alliance = Optional.empty();

  public static void UpdateSettings() {
    Constants.alliance = DriverStation.getAlliance();
  }
}
