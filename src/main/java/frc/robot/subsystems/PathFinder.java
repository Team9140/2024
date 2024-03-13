package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;

import java.util.List;

public class PathFinder {
    private static PathFinder instance;
    private final Drivetrain drive;
    private Arm arm;
    private Thrower thrower;
    private Intake intake;

    public static PathFinder getInstance() {
        return PathFinder.instance == null ? PathFinder.instance = new PathFinder() : PathFinder.instance;
    }
    public PathFinder(){
        this.drive = Drivetrain.getInstance();
        this.arm = Arm.getInstance();
        this.thrower = Thrower.getInstance();
        this.intake = Intake.getInstance();

        configPathPlanner();
    }

    public void configPathPlanner(){
        AutoBuilder.configureHolonomic(
                drive::getPosition,
                drive::resetPosition,
                drive::getSpeed,
                drive::swerveDrive,
                new HolonomicPathFollowerConfig(
                        Constants.Drivetrain.METERS_PER_SECOND,
                        Units.inchesToMeters(Math.hypot(Constants.WIDTH, Constants.LENGTH) / 2),
                        new ReplanningConfig()
                ),
                () -> Constants.alliance.isPresent() && Constants.alliance.get() == DriverStation.Alliance.Red,
                drive
        );
    }
    public void auto(){
        Command overhandLaunchCommand =
                this.arm.setOverhand() // Set overhand aim
                        .alongWith(this.thrower.prepareSpeaker()) // Start launchers before throwing
                        .andThen(new WaitUntilCommand(this.arm::isReady)) // Wait until the launchers are spinning fast enough
                        .andThen(this.thrower.launch()) // Launch
                        .andThen(this.thrower.setIntake().alongWith(this.arm.setIntake())); // Adjust intake along with arm

        Command floorIntake =
                new WaitUntilCommand(this.arm::isReady) // Wait until the launchers are spinning fast enough
                        .andThen(this.intake.intakeNote()); // Start floor intake

        NamedCommands.registerCommand("Intake", floorIntake);
        NamedCommands.registerCommand("Intake_Off", this.intake.off());
        NamedCommands.registerCommand("Shoot", overhandLaunchCommand);

        List<PathPlannerPath> pathGroup = PathPlannerAuto.getPathGroupFromAutoFile("Auto1");
        PathPlannerPath path1 = pathGroup.get(0);
        PathPlannerPath path2 = pathGroup.get(1);
        PathPlannerPath path3 = pathGroup.get(2);
        PathPlannerPath path4 = pathGroup.get(3);
        PathPlannerPath path5 = pathGroup.get(4);
        PathPlannerPath path6 = pathGroup.get(5);

        new SequentialCommandGroup(
                overhandLaunchCommand,
                AutoBuilder.followPath(path1),
                AutoBuilder.followPath(path2),
                AutoBuilder.followPath(path3),
                AutoBuilder.followPath(path4),
                AutoBuilder.followPath(path5),
                AutoBuilder.followPath(path6)
        );
    }

    public Command autoToBlueAmp(){
        configPathPlanner();
        PathPlannerPath blueAmpPath = PathPlannerPath.fromPathFile("BlueAmp");
        PathConstraints constraints = new PathConstraints(
                3.0, 4.0,
                Units.degreesToRadians(540), Units.degreesToRadians(720));
        return AutoBuilder.pathfindThenFollowPath(
                blueAmpPath,
                constraints,
                3.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
        );
    }


}
