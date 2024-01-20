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

//  public static final double fieldx = Units.inchesToMeters(501);
//  public static final double fieldy = Units.inchesToMeters(323.28);
  public static final int kencoderCountsPerRev = 0;

  public static final class Drivetrain {
    public static final int frontLeftDrivePort = 1;
    public static final int frontLeftTurnPort = 1;
    public static final int frontRightDrivePort = 1;
    public static final int frontRightTurnPort = 1;
    public static final int backLeftDrivePort = 1;
    public static final int backLeftTurnPort = 1;
    public static final int backRightDrivePort = 1;
    public static final int backRightTurnPort = 1;
  }

  public static class Ports {
    public static final int INPUT_CONTROLLER = 0;
    public static final int GYRO = 0;
  }

  public static Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();

  public static void UpdateSettings() {
    Constants.alliance = DriverStation.getAlliance();
  }
}
