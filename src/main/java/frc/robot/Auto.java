package frc.robot;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoControlFunction;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Thrower;

public class Auto {
  private final Drivetrain drive;
  private final Arm arm;
  private final Thrower thrower;
  private final Intake intake;
  private int autoChoice = Constants.DEFAULT_AUTO;
  private final ChoreoControlFunction choreoPID;
  private final SendableChooser<Integer> autonomousSendableChooser = new SendableChooser<>();

  public Auto() {
    SmartDashboard.putString("Current Auto", "Program resetting...");
    this.drive = Drivetrain.getInstance();
    this.arm = Arm.getInstance();
    this.thrower = Thrower.getInstance();
    this.intake = Intake.getInstance();

    NamedCommands.registerCommand("e1", this.intakeOnCommand());
    NamedCommands.registerCommand("e2", this.armOverhandCommand());
    NamedCommands.registerCommand("e3", this.launchCommand());
    NamedCommands.registerCommand("e4", this.intakeOnCommand());
    NamedCommands.registerCommand("e5", this.armOverhandCommand());
    NamedCommands.registerCommand("e6", this.launchCommand());
    NamedCommands.registerCommand("e7", this.intakeOnCommand());
    NamedCommands.registerCommand("e8", this.armOverhandCommand());
    NamedCommands.registerCommand("e9", this.launchCommand());

    this.choreoPID = Choreo.choreoSwerveController(
      new PIDController(1.0, 0.0, 0.0),
      new PIDController(1.0, 0.0, 0.0),
      new PIDController(1.0, 0.0, 0.0)
    );

    AutoBuilder.configureHolonomic(
      this.drive::getPosition,
      this.drive::resetPosition,
      this.drive::getSpeed,
      this.drive::swerveDrive,
      new HolonomicPathFollowerConfig(
        new PIDConstants(1.0, 0.0, 0.0),
        new PIDConstants(1.0, 0.0, 0.0),
        Constants.Drivetrain.LINEAR_VELOCITY,
        Units.inchesToMeters(Constants.BASE_RADIUS),
        new ReplanningConfig()
      ),
      () -> Constants.alliance.isPresent() && Constants.alliance.get() == DriverStation.Alliance.Red,
      this.drive
    );

    this.recursiveAddAutos("Auto", Constants.REGULAR_AUTOS, 0);
    this.recursiveAddAutos("Choreo", Constants.CHOREO_AUTOS.keySet().toArray(), Constants.CHOREO_AUTOS_OFFSET);
    this.recursiveAddAutos("PathPlanner", Constants.PATHPLANNER_AUTOS.keySet().toArray(), Constants.PATHPLANNER_AUTOS_OFFSET);
    SmartDashboard.putData("Autos", this.autonomousSendableChooser);
  }

  private void recursiveAddAutos(String name, Object[] array, int offset) {
    for (int i = 0; i < array.length; i++) {
      if (i + offset == Constants.DEFAULT_AUTO) {
        this.autonomousSendableChooser.setDefaultOption("[" + name + "] " + array[i].toString() + " (Default)", i + offset);
      } else {
        this.autonomousSendableChooser.addOption("[" + name + "] " + array[i].toString(), i + offset);
      }
    }
  }

  /**
   * Sets the autoChoice to the selected auto on SmartDashboard
   * @return The selected autonomous choice
   **/
  public int getSelectedAutoId() {
    Integer choice = this.autonomousSendableChooser.getSelected();
    return this.autoChoice = choice == null ? Constants.DEFAULT_AUTO : choice;
  }

  /**
   * The name of the current auto
   * @return The friendly human-readable name of the currently selected autonomous mode
   **/
  public String getAutoName(int choice) {
    if (choice >= Constants.PATHPLANNER_AUTOS_OFFSET) {
      return Constants.PATHPLANNER_AUTOS.keySet().toArray()[choice - Constants.PATHPLANNER_AUTOS_OFFSET].toString();
    } else if (choice >= Constants.CHOREO_AUTOS_OFFSET) {
      return Constants.CHOREO_AUTOS.keySet().toArray()[choice - Constants.CHOREO_AUTOS_OFFSET].toString();
    } else {
      return Constants.REGULAR_AUTOS[choice];
    }
  }

  /**
    * Gets the auto command from a specified autonomous Id
    * @param choice The Id number
    * @return The command for the auto mode
   **/
  public Command getAutoCommand(int choice) {
    if (choice >= Constants.CHOREO_AUTOS_OFFSET) {
      return this.getChoreoPath(Constants.CHOREO_AUTOS.get(Constants.CHOREO_AUTOS.keySet().toArray()[choice].toString()));
    } else {
      return this.getPathPlannerPath(Constants.PATHPLANNER_AUTOS.keySet().toArray()[this.autoChoice].toString());
    }
  }

  /**
    * Gets a command that runs a Choreo path from a specified name
    * @param path The filename of the requested path
    * @return A command that follows the path
   **/
  public Command getChoreoPath(String path) {
    return Choreo.choreoSwerveCommand(
      Choreo.getTrajectory(path),
      this.drive::getPosition,
      this.choreoPID,
      this.drive::swerveDrive,
      () -> Constants.alliance.isPresent() && Constants.alliance.get() == DriverStation.Alliance.Red,
      this.drive
    );
  }

  /**
   * Gets a command that runs a PathPlanner path from a specified name
   * @param path The filename of the requested path
   * @return A command that follows the path
   **/
  public Command getPathPlannerPath(String path) {
    return AutoBuilder.followPath(PathPlannerPath.fromPathFile(path));
  }

  /**
   * Finds a path to the requested position
   * @param endPos The requested final position of the bot
   * @return A command that finds a path to the requested position
   **/
  public Command findPathToPosition(Pose2d endPos) {
    return AutoBuilder.pathfindToPose(endPos, getPathPlannerConstraints());
  }

  /**
   * Get constraints for PathPlanner
   * @return The path constraints
   **/
  private PathConstraints getPathPlannerConstraints() {
    return new PathConstraints(
      3.0, 4.0,
      Units.degreesToRadians(540), Units.degreesToRadians(720)
    );
  }


  /**
    * Turns off the intake and sets the arm to the overhand position
    * @return A command that moves the arm to the overhand position
   **/
  public Command armOverhandCommand() {
    return this.arm.setOverhand().alongWith(this.thrower.prepareSpeaker()).alongWith(this.intake.off()); // Adjust intake along with arm
  }

  /**
    * Launch a note then moves the arm to the stow position
    * @return A command that launches the note
   **/
  public Command launchCommand() {
    return new SequentialCommandGroup(
      this.thrower.launch(),
      new WaitCommand(0.25),
      this.arm.setStow()
    );
  }

  /**
    * Turns on the intake motors
    * @return A command that turns on the intake motors
   **/
  public Command intakeOnCommand() {
    return this.intake.intakeNote().alongWith(this.arm.setIntake()).alongWith(this.thrower.setIntake());
  }

  /**
    * Turns off the intake motors
    * @return A command that turns off the intake motors
   **/
  public Command intakeOffCommand() {
    return this.thrower.off().alongWith(this.intake.off());
  }
}
