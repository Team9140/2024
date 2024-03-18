package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;

public class Path {
    private static Path instance;
    private final Drivetrain drive;
    private final Arm arm;
    private final Thrower thrower;
    private final Intake intake;

    public static Path getInstance() {
        return Path.instance == null ? Path.instance = new Path() : Path.instance;
    }

    private Path() {
        this.drive = Drivetrain.getInstance();
        this.arm = Arm.getInstance();
        this.thrower = Thrower.getInstance();
        this.intake = Intake.getInstance();

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

    public Command getOverhandLaunch(double customPosition){
        return new SequentialCommandGroup(
                this.arm.setAngle(Constants.Arm.Positions.OVERHAND + customPosition) // Set overhand aim
                        .andThen(this.thrower.prepareSpeaker()) // Start launchers before throwing
                        .andThen(new WaitUntilCommand(this.arm::isReady)) // Wait until the launchers are spinning fast enough
                        .andThen(this.thrower.launch()) //launch
                        .andThen(new WaitCommand(0.5)) //wait
                        .andThen(this.thrower.off())
                        .andThen(this.arm.setStow())); // Adjust intake along with arm
    }

    public Command getIntakeOn(){
        return new SequentialCommandGroup(
                this.thrower.setIntake().alongWith(this.arm.setIntake())
                        .andThen(new WaitUntilCommand(this.arm::isReady)) // wait until the launchers are spinning fast enough
                        .andThen(this.intake.intakeNote()));
    }

    public Command getIntakeOff(){
        return new SequentialCommandGroup(this.thrower.off().alongWith(this.intake.off()));
    }
    public Command auto() {

        PathPlannerPath path1 = PathPlannerPath.fromPathFile("Path1");
        PathPlannerPath path2 = PathPlannerPath.fromPathFile("Path2");
        PathPlannerPath path3 = PathPlannerPath.fromPathFile("Path3");

        return new SequentialCommandGroup(
                getOverhandLaunch(0),
                getIntakeOn(),
                AutoBuilder.followPath(path1),
                getIntakeOff(),
                getOverhandLaunch(Math.toRadians(0)),
                getIntakeOn(),
                AutoBuilder.followPath(path2),
                getIntakeOff(),
                getOverhandLaunch(Math.toRadians(15.0)),
                getIntakeOn(),
                AutoBuilder.followPath(path3),
                getIntakeOff(),
                getOverhandLaunch(Math.toRadians(15.0))
        );
    }

    public void pathFindToPose(Pose2d endPos) {
        AutoBuilder.pathfindToPose(endPos, getPathConstraints());
    }
}
