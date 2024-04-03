package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Globals {
  // Side of the field per-match
  public static DriverStation.Alliance alliance = DriverStation.Alliance.Blue;
  public static Pose2d STARTING_POSITION;

  private static final SendableChooser<Integer> positionChooser = new SendableChooser<>();

  public static void addStartingPositions() {
    Object[] startingPositions = Constants.Auto.POSITIONS.keySet().toArray();

    for (int i = 0; i < startingPositions.length; i++) {
      if (i == Constants.Auto.DEFAULT_POSITION) {
        Globals.positionChooser.setDefaultOption("[Position] " + startingPositions[i].toString() + " (Default)", i);
      } else {
        Globals.positionChooser.addOption("[Position] " + startingPositions[i].toString(), i);
      }
    }

    SmartDashboard.putData("Positions", Globals.positionChooser);
  }

  public static void UpdateSettings() {
    Globals.alliance = DriverStation.getAlliance().orElseGet(() -> DriverStation.Alliance.Blue);
    Integer position = Globals.positionChooser.getSelected();

    if (position != null) {
      String positionString = Constants.Auto.POSITIONS.keySet().toArray()[position].toString();

      if (Constants.Auto.POSITIONS.containsKey(positionString)) {
        Pose2d startingPosition = Constants.Auto.POSITIONS.get(positionString);
        Globals.STARTING_POSITION = Globals.alliance == DriverStation.Alliance.Red
          ? new Pose2d(startingPosition.getX(), startingPosition.getY(), Rotation2d.fromDegrees(startingPosition.getRotation().getDegrees() * -1))
          : startingPosition;
      } else {
        System.out.println("[ WARN ] The starting position was not updated properly: '" + positionString + "'");
      }
      SmartDashboard.putString("Auto Starting Position", positionString);
    } else {
      // Set the starting position to the default starting position if it cannot read a value from SmartDashboard
      System.out.println("[ WARN ] The starting position was not updated properly");
      Globals.STARTING_POSITION = Constants.Auto.POSITIONS.get(Constants.Auto.POSITIONS.keySet().toArray()[Constants.Auto.DEFAULT_POSITION]);
    }

    SmartDashboard.putString("Auto Starting Coords", Globals.STARTING_POSITION.toString());
    SmartDashboard.putString("Alliance", Globals.alliance.toString());
  }
}
