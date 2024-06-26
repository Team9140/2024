package frc.robot;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ChoreoPathCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Thrower;

import java.util.HashMap;
import java.util.function.Supplier;

public class Auto {
  private final Drivetrain drive;
  private final Arm arm;
  private final Thrower thrower;
  private final Intake intake;
  private final SendableChooser<Integer> autonomousSendableChooser = new SendableChooser<>();
  private static final HashMap<String, ChoreoTrajectory> trajectories = new HashMap<>();
  public final ChoreoPathCommand tempAutoCommand;  // TODO: implement properly

  public Auto() {
    SmartDashboard.putString("Current Auto", "Program resetting...");
    this.drive = Drivetrain.getInstance();
    this.arm = Arm.getInstance();
    this.thrower = Thrower.getInstance();
    this.intake = Intake.getInstance();

    AutoBuilder.configureHolonomic(
      this.drive::getPosition,
      this.drive::resetPosition,
      this.drive::getChassisSpeeds,
      this.drive::swerveDrive,
      new HolonomicPathFollowerConfig(
        new PIDConstants(1.0, 0.0, 0.0),
        new PIDConstants(1.0, 0.0, 0.0),
        Constants.Drivetrain.LINEAR_VELOCITY,
        Units.inchesToMeters(Constants.Auto.BASE_RADIUS),
        new ReplanningConfig()
      ),
      () -> Globals.alliance == DriverStation.Alliance.Red,
      this.drive
    );

    this.recursiveAddAutos("Auto", Constants.Auto.REGULAR, 0);
    this.recursiveAddAutos("Choreo", Constants.Auto.CHOREO.keySet().toArray(), Constants.Auto.CHOREO_OFFSET);
    this.recursiveAddAutos("PathPlanner", Constants.Auto.PATHPLANNER.keySet().toArray(), Constants.Auto.PATHPLANNER_OFFSET);
    this.autonomousSendableChooser.addOption("[Disabled] DISABLE AUTONOMOUS", Constants.Auto.DISABLED_ID);
    SmartDashboard.putData("Autos", this.autonomousSendableChooser);

    Auto.NamedCommands.addCommand("intake", this::intakeCommand);
    Auto.NamedCommands.addCommand("launch", this.thrower::launch);
    Auto.NamedCommands.addCommand("overhand", this::armOverhandCommand);
    Auto.NamedCommands.addCommand("speakerShot", () -> this.armPositionCommand(Constants.Arm.Positions.OVERHAND + 0.125));

    this.tempAutoCommand = this.getChoreoPath("testStraightAuto");
  }

  private void recursiveAddAutos(String name, Object[] array, int offset) {
    for (int i = 0; i < array.length; i++) {
      if (i + offset == Constants.Auto.DEFAULT_MODE) {
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
    return choice == null ? Constants.Auto.DEFAULT_MODE : choice;
  }

  /**
   * The name of the current auto
   * @return The friendly human-readable name of the currently selected autonomous mode
   **/
  public String getAutoName(int choice) {
    if (choice >= Constants.Auto.PATHPLANNER_OFFSET) {
      return Constants.Auto.PATHPLANNER.keySet().toArray()[choice - Constants.Auto.PATHPLANNER_OFFSET].toString();
    } else if (choice >= Constants.Auto.CHOREO_OFFSET) {
      return Constants.Auto.CHOREO.keySet().toArray()[choice - Constants.Auto.CHOREO_OFFSET].toString();
    } else {
      return Constants.Auto.REGULAR[choice];
    }
  }

  /**
    * Gets the auto command from a specified autonomous Id
    * @return The command for the auto mode
   **/
  public Command getAutoCommand() {
    System.out.println(Timer.getFPGATimestamp() + " update settings");
    Globals.UpdateSettings();
    System.out.println(Timer.getFPGATimestamp() + " get selected auto id");
    int autoChoice = this.getSelectedAutoId();
    System.out.println(Timer.getFPGATimestamp() + " check if disabled");
    if (autoChoice == Constants.Auto.DISABLED_ID) return new InstantCommand(() -> System.out.println("auto disabled"));
    System.out.println(Timer.getFPGATimestamp() + " main switch case");
    if (autoChoice < Constants.Auto.CHOREO_OFFSET) {
      return (switch (autoChoice) {
        default:
          System.out.println("Error: autoChoice value " + autoChoice + " is invalid. Defaulting to 0.");
        case 0:  // Shoot & Drive
          yield new SequentialCommandGroup(
            this.arm.setOverhand().alongWith(this.thrower.prepareSpeaker()),
            new WaitCommand(1),
            this.thrower.launch(),
            this.thrower.off().alongWith(this.arm.setStow()),
            this.drive.goStraight(1, 2)
          );
        case 1:  // 4-Note
          yield new SequentialCommandGroup(
            new InstantCommand(() -> System.out.println(Timer.getFPGATimestamp() + " starting auto")),
            this.armOverhandCommand(),
            new WaitCommand(0.5),
            this.thrower.launch(),
            new WaitCommand(0.25),
            this.arm.setStow().alongWith(this.intake.off()),
            new WaitCommand(0.25),
            this.tempAutoCommand.alongWith(new SequentialCommandGroup(
              this.intake.intakeNote().alongWith(this.thrower.setIntake()),
              new WaitCommand(2.2),
              this.armPositionCommand(Constants.Arm.Positions.OVERHAND + 0.125),
              new WaitCommand(0.85),
              this.thrower.launch(),  // throw 1
              new WaitCommand(0.1),
              this.intakeCommand(),
              new WaitCommand(1.45),
              this.armPositionCommand(Constants.Arm.Positions.OVERHAND + 0.125),
              new WaitCommand(0.7),
              this.thrower.launch(),  // throw 2
              new WaitCommand(0.2),
              this.intakeCommand(),
              new WaitCommand(1.7),
              this.armPositionCommand(0.25 * Math.PI + 0.1),
              new WaitCommand(0.7),
              this.thrower.launch(),  // throw 3
              new WaitCommand(0.2),
              this.thrower.off().alongWith(this.arm.setStow()))
            )/*,
            this.getChoreoPath("5note").alongWith(new SequentialCommandGroup(
              new WaitCommand(2.0),
              this.intake.intakeNote().alongWith(this.thrower.setIntake()),
              new WaitCommand(1.5),
              this.intake.off().alongWith(this.thrower.off()),
              new WaitCommand(1.5),
              this.thrower.prepareSpeaker().alongWith(this.arm.setOverhand()),
              new WaitCommand(1.0),
              this.thrower.launch(),
              new WaitCommand(0.2),
              this.thrower.off().alongWith(this.arm.setStow())
            ))*/
          );
        case 2:
          yield new SequentialCommandGroup(
            this.arm.setOverhand().alongWith(this.thrower.prepareSpeaker()),
            new WaitCommand(1),
            this.thrower.launch(),
            new WaitCommand(0.2),
            this.thrower.off().alongWith(this.arm.setStow())
          );
      });
    } else if (autoChoice < Constants.Auto.PATHPLANNER_OFFSET) {
      return this.getChoreoPath(Constants.Auto.PATHPLANNER.get(Constants.Auto.CHOREO.keySet().toArray()[autoChoice - Constants.Auto.CHOREO_OFFSET].toString()));
    } else {
      return this.getPathPlannerPath(Constants.Auto.PATHPLANNER.keySet().toArray()[autoChoice - Constants.Auto.PATHPLANNER_OFFSET].toString());
    }
  }

  /**
    * Gets a command that runs a Choreo path from a specified name
    * @param path The filename of the requested path
    * @return A command that follows the path
   **/
  public ChoreoPathCommand getChoreoPath(String path) {
    System.out.println(Timer.getFPGATimestamp() + " create choreo path");
    ChoreoPathCommand command = new ChoreoPathCommand(path);
    /*System.out.println(Timer.getFPGATimestamp() + " read file");
    Scanner file;

    File folder = new File("home/lvuser");
    for (File f : folder.listFiles()) {
      System.out.println(f.getName());
    }
    System.out.println(Timer.getFPGATimestamp() + " check path file");
    try {
//      file = new Scanner(new File("choreo/" + path + ".traj"));
      file = new Scanner(new File("src/main/deploy/choreo/" + path + ".traj"));
    } catch (FileNotFoundException e) {
      e.printStackTrace();
      return command;
    }

    System.out.println(Timer.getFPGATimestamp() + " parse json");
    StringBuilder JSONString = new StringBuilder();
    while (file.hasNextLine()) {
      JSONString.append(file.nextLine());
    }

    JSONObject json;
    try {
      json = (JSONObject) (new JSONParser()).parse(JSONString.toString());
    } catch (ParseException e) {
      e.printStackTrace();
      return command;
    }

    System.out.println(Timer.getFPGATimestamp() + " get event markers");

    for (Object o : (JSONArray) json.get("eventMarkers")) {
      JSONObject event = (JSONObject) o;
      Set<String> keys = event.keySet();
      if (!keys.contains("command")) {
        continue;
      }

      JSONObject eventCommand = (JSONObject) event.get("command");
      if (!eventCommand.get("type").equals("named")) {
        continue;
      }

      Double timestamp = (Double) event.get("timestamp");
      if (timestamp == null) {
        continue;
      }

      command.addEventMarker(timestamp, (String) ((JSONObject) eventCommand.get("data")).get("name"));
    }*/

    return command;
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
   * @param end The requested final position of the bot
   * @return A command that finds a path to the requested position
   **/
  public Command findPathToPosition(Pose2d end) {
    return AutoBuilder.pathfindToPose(end, new PathConstraints(
      3.0, 4.0,
      Units.degreesToRadians(540), Units.degreesToRadians(720)
    ));
  }

  /**
    * Sets the arm to the overhand position and prepares to launch in the speaker
    * @return A command that moves the arm and prepares to launch
   **/
  public Command armOverhandCommand() {
    return this.intake.off().alongWith(this.thrower.prepareSpeaker()).alongWith(this.arm.setOverhand());
  }

  /**
    * Sets the arm to a specified position and prepares to launch in the speaker
    * @param angle The requested angle in radians
    * @return A command that moves the arm and prepares to launch
   **/
  public Command armPositionCommand(double angle) {
    return this.intake.off().alongWith(this.thrower.prepareSpeaker()).alongWith(this.arm.setAngle(angle));
  }

  /**
    * Turns on the intake motors
    * @return A command that turns on the intake motors
   **/
  public Command intakeCommand() {
    return this.intake.intakeNote().alongWith(this.thrower.setIntake()).alongWith(this.arm.setStow());
  }

  public static ChoreoTrajectory getChoreoTrajectory(String name) {
    if (Auto.trajectories.containsKey(name)) {
      return Auto.trajectories.get(name);
    } else {
      ChoreoTrajectory trajectory = Choreo.getTrajectory(name);
      Auto.trajectories.put(name, trajectory);
      return trajectory;
    }
  }

  public static class NamedCommands {
    private static final HashMap<String, Supplier<Command>> commands = new HashMap<>();

    public static Command getCommand(String name) {
      if (Auto.NamedCommands.commands.containsKey(name)) {
        return Auto.NamedCommands.commands.get(name).get();
      } else {
        return new InstantCommand(() -> System.out.println("Scheduled command " + name + " could not be found."));
      }
    }

    public static void addCommand(String name, Supplier<Command> command) {
      Auto.NamedCommands.commands.put(name, command);
    }
  }
}
