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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;

public class PathFinder {
    private static PathFinder instance;
    private final Drivetrain drive;
    private final Arm arm;
    private final Thrower thrower;
    private final Intake intake;

    public static PathFinder getInstance() {
        return PathFinder.instance == null ? PathFinder.instance = new PathFinder() : PathFinder.instance;
    }

    private PathFinder() {
        this.drive = Drivetrain.getInstance();
        this.arm = Arm.getInstance();
        this.thrower = Thrower.getInstance();
        this.intake = Intake.getInstance();
        configPathPlanner();
    }

    public void configPathPlanner() {
        AutoBuilder.configureHolonomic(
                drive::getPosition,
                drive::resetPosition,
                drive::getSpeed,
                drive::swerveDrive,
                new HolonomicPathFollowerConfig(
                        new PIDConstants(1.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(1.0, 0.0, 0.0),
                        Constants.Drivetrain.METERS_PER_SECOND,
                        Units.inchesToMeters(Constants.BASE_RADUS),
                        new ReplanningConfig()
                ),
                () -> Constants.alliance.isPresent() && Constants.alliance.get() == DriverStation.Alliance.Red,
                drive
        );
    }

    public PathConstraints getPathConstraints() {
        return new PathConstraints(
                3.0, 4.0,
                Units.degreesToRadians(540), Units.degreesToRadians(720));
    }

    public void auto() {
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


        PathPlannerPath path = PathPlannerPath.fromPathFile("Path1");
        AutoBuilder.followPath(path).schedule();

    }

    public void pathFindToPose(Pose2d endPos) {
        configPathPlanner();
        AutoBuilder.pathfindToPose(endPos, getPathConstraints());
    }
}
