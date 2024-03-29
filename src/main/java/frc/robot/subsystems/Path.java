package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;

import java.util.HashMap;
import java.util.Map;

public class Path extends SubsystemBase {
  private final Drivetrain drive;
  private final Arm arm;
  private final Thrower thrower;
  private final Intake intake;
  private int autoChoice = Constants.DEFAULT_AUTO;
  private final HashMap<String, PathPlannerPath> autoPaths;
  private final SendableChooser<Integer> autos = new SendableChooser<>();

  public Path() {
    SmartDashboard.putString("Current Auto", "Program resetting...");
    this.drive = Drivetrain.getInstance();
    this.arm = Arm.getInstance();
    this.thrower = Thrower.getInstance();
    this.intake = Intake.getInstance();

    NamedCommands.registerCommand("e1", this.getIntakeOn());
    NamedCommands.registerCommand("e2", this.prepareOverhand());
    NamedCommands.registerCommand("e3", this.launch());
    NamedCommands.registerCommand("e4", this.getIntakeOn());
    NamedCommands.registerCommand("e5", this.prepareOverhand());
    NamedCommands.registerCommand("e6", this.launch());
    NamedCommands.registerCommand("e7", this.getIntakeOn());
    NamedCommands.registerCommand("e8", this.prepareOverhand());
    NamedCommands.registerCommand("e9", this.launch());

    this.autoPaths = new HashMap<>(Map.ofEntries(
//      Map.entry("Blue Amp Side", PathPlannerPath.fromPathFile("BlueAmpSideTriple")),  // NOT WORKING
//      Map.entry("Blue Mid Side", PathPlannerPath.fromPathFile("BlueMidSideTriple")),  // NOT WORKING
//      Map.entry("Blue Ref Side", PathPlannerPath.fromPathFile("BlueRefSideTriple")),  // NOT WORKING
//      Map.entry("Blue Leave Source", PathPlannerPath.fromPathFile("LEAVE"))//,
//      Map.entry("Red Amp Side", PathPlannerPath.fromPathFile("RedAmpSideTriple")),
    ));

    AutoBuilder.configureHolonomic(
      this.drive::getPosition,
      this.drive::resetPosition,
      this.drive::getSpeed,
      this.drive::swerveDrive,
      new HolonomicPathFollowerConfig(
        new PIDConstants(1.0, 0.0, 0.0), // Translation PID constants
        new PIDConstants(1.0, 0.0, 0.0),
        Constants.Drivetrain.LINEAR_VELOCITY,
        Units.inchesToMeters(Constants.BASE_RADIUS),
        new ReplanningConfig()
      ),
      () -> Constants.alliance.isPresent() && Constants.alliance.get() == DriverStation.Alliance.Red,
      this.drive
    );

  }


  /**
    * Routinely updates the selected autonomous choice
   **/
  @Override
  public void periodic() {
    this.getAutoChoice();
  }

  /**
    * Get the path constraints for the robot. Contains information such as max velocity and acceleration
    * @return The path constraints
   **/
  public PathConstraints getPathConstraints() {
    return new PathConstraints(
      3.0, 4.0,
      Units.degreesToRadians(540), Units.degreesToRadians(720)
    );
  }

  /**
    * Prepares overhand.
    * @return A command that sets the angle and launcher motors for launching, along with turning off the intake.
   **/
  public Command prepareOverhand(){
    return this.arm.setOverhand().alongWith(this.thrower.prepareSpeaker()).alongWith(this.intake.off()); // Adjust intake along with arm
  }

  /**
    * Sets up launching.
    * @return A command that gives the note to the launch motors, launching the note. It then stows the arm after 1/4 second.
   **/
  public Command launch() {
    return new SequentialCommandGroup(
      this.thrower.launch(),
      new WaitCommand(0.25),
      this.arm.setStow()
    );
  }

  /**
    * Sets the intake on for all required motors
    * @return A command that turns all required intake motors to intake required voltage.
   **/
  public Command getIntakeOn(){
    return this.intake.intakeNote().alongWith(this.arm.setIntake()).alongWith(this.thrower.setIntake());
  }

  /**
    * Turns off all intake required motors
    * @return A command that turns all required intake motors off
   **/
  public Command getIntakeOff(){
    return this.thrower.off().alongWith(this.intake.off());
  }

  /**
    * Tells the robor to follow the path given by the name.
    * @param path The filename of the requested path
    * @return A command that follows the path.
   **/
  public Command auto(String path) {
    return AutoBuilder.followPath(this.autoPaths.get(path));
  }

  /**
    * Follows the autonomous path that is selected
    * @return A command that follows the path
   **/
  public Command auto() {
    return this.auto(this.autoPaths.keySet().toArray()[this.autoChoice].toString());
  }

  /**
    * Places debug information on SmartDashbord
   **/
  public void autoChooser() {
    for (int i = 0; i < this.autoPaths.size(); i++) {
      if (i == Constants.DEFAULT_AUTO) {
        this.autos.setDefaultOption("[Path] " + this.autoPaths.keySet().toArray()[i] + " (Default)", i);
      } else {
        this.autos.addOption("[Path] " + this.autoPaths.keySet().toArray()[i], i);
      }
    }

    for (int i = 0; i < Constants.REGULAR_AUTOS.length; i++) {
      if (i + Constants.REGULAR_AUTOS_OFFSET == Constants.DEFAULT_AUTO) {
        this.autos.setDefaultOption("[Auto] " + Constants.REGULAR_AUTOS[i] + " (Default)", i + Constants.REGULAR_AUTOS_OFFSET);
      } else {
        this.autos.addOption("[Auto] " + Constants.REGULAR_AUTOS[i], i + Constants.REGULAR_AUTOS_OFFSET);
      }
    }

    SmartDashboard.putData("Autos", this.autos);
  }

  /**
    * Sets the autoChoice to the selected auto on SmartDashboard
    * @return The selected autonomous choice
   **/
  public int getAutoChoice() {
    Integer choice = this.autos.getSelected();
    this.autoChoice = choice == null ? Constants.DEFAULT_AUTO : choice;

    SmartDashboard.putString("Current Auto", this.autoChoice < Constants.REGULAR_AUTOS_OFFSET ? this.autoPaths.keySet().toArray()[this.autoChoice].toString() : Constants.REGULAR_AUTOS[this.autoChoice - Constants.REGULAR_AUTOS_OFFSET]);
    return this.autoChoice;
  }

  /**
    * Finds a path to the requested position
    * @param endPos The requested final position of the bot
    * @return A command that pathfinds to the requested position
   **/
  public Command pathFindToPose(Pose2d endPos) {
    return AutoBuilder.pathfindToPose(endPos, getPathConstraints());
  }
}
