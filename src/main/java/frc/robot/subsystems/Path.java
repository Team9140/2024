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
  private static Path instance;
  private final Drivetrain drive;
  private final Arm arm;
  private final Thrower thrower;
  private final Intake intake;
  private int autoChoice = Constants.DEFAULT_AUTO;

  private final HashMap<String, PathPlannerPath> autoPaths;

  private SendableChooser<Integer> autos = new SendableChooser<>();

  public static Path getInstance() {
    return Path.instance == null
      ? Path.instance = new Path()
      : Path.instance;
  }

  private Path() {
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
//    NamedCommands.registerCommand("prepareLaunch", this.getPrepareOverhandLaunch());
//    NamedCommands.registerCommand("launch", this.getOverhandLaunch());
//    NamedCommands.registerCommand("intake", this.getIntakeOn());
//    NamedCommands.registerCommand("intakeOff", this.getIntakeOff());

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
        Constants.Drivetrain.METERS_PER_SECOND,
        Units.inchesToMeters(Constants.BASE_RADUS),
        new ReplanningConfig()
      ),
      () -> Constants.alliance.isPresent() && Constants.alliance.get() == DriverStation.Alliance.Red,
      this.drive
    );

  }

  @Override
  public void periodic() {
    this.getAutoChoice();
  }

  public PathConstraints getPathConstraints() {
    return new PathConstraints(
      3.0, 4.0,
      Units.degreesToRadians(540), Units.degreesToRadians(720)
    );
  }

  public Command prepareOverhand(){
    return this.arm.setOverhand().alongWith(this.thrower.prepareSpeaker()).alongWith(this.intake.off()); // Adjust intake along with arm
  }

  public Command launch() {
    return new SequentialCommandGroup(
      this.thrower.launch(),
      new WaitCommand(0.25),
      this.arm.setStow()
    );
  }

  public Command getIntakeOn(){
    return this.intake.intakeNote().alongWith(this.arm.setIntake()).alongWith(this.thrower.setIntake());
  }

  public Command getIntakeOff(){
    return this.thrower.off().alongWith(this.intake.off());
  }

  public Command auto(String path) {
    return AutoBuilder.followPath(this.autoPaths.get(path));
  }

  public Command auto() {
    return this.auto(this.autoPaths.keySet().toArray()[this.autoChoice].toString());
  }

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

  public int getAutoChoice() {
    Integer choice = this.autos.getSelected();
    this.autoChoice = choice == null ? Constants.DEFAULT_AUTO : choice;

    SmartDashboard.putString("Current Auto", this.autoChoice < Constants.REGULAR_AUTOS_OFFSET ? this.autoPaths.keySet().toArray()[this.autoChoice].toString() : Constants.REGULAR_AUTOS[this.autoChoice - Constants.REGULAR_AUTOS_OFFSET]);
    return this.autoChoice;
  }

  public void pathFindToPose(Pose2d endPos) {
    AutoBuilder.pathfindToPose(endPos, getPathConstraints());
  }
}
