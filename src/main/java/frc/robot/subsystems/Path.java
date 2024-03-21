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

  private PathPlannerPath autoPath;
  public static Path getInstance() {
    return Path.instance == null ? Path.instance = new Path() : Path.instance;
  }

  private Path() {
    this.drive = Drivetrain.getInstance();
    this.arm = Arm.getInstance();
    this.thrower = Thrower.getInstance();
    this.intake = Intake.getInstance();

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

  public PathConstraints getPathConstraints() {
    return new PathConstraints(
            3.0, 4.0,
            Units.degreesToRadians(540), Units.degreesToRadians(720)
    );
  }

  public Command getPrepareOverhandLaunch() {
    return new SequentialCommandGroup(
            this.arm.setAngle(Constants.Arm.Positions.OVERHAND), // Set overhand aim
            this.thrower.prepareSpeaker(),
            new WaitUntilCommand(this.arm::isReady)
    ); // Wait until the launchers are spinning fast enough
  }
  public Command getOverhandLaunch(){
    return new SequentialCommandGroup(
            this.thrower.launch(), // Launch
            new WaitCommand(0.5), // Wait
            this.thrower.off(),
            this.arm.setStow()
    ); // Adjust intake along with arm
  }

  public Command getIntakeOn(){
    return new SequentialCommandGroup(
            this.thrower.setIntake().alongWith(this.arm.setIntake()),
            new WaitUntilCommand(this.arm::isReady), // wait until the launchers are spinning fast enough
            this.intake.intakeNote()
    );
  }

  public Command getIntakeOff(){
    return this.thrower.off().alongWith(this.intake.off());
  }

  public Command auto() {
    autoPath = Constants.AUTO_PATH;
    return AutoBuilder.followPath(autoPath);

  }

  public void pathFindToPose(Pose2d endPos) {
    AutoBuilder.pathfindToPose(endPos, getPathConstraints());
  }
}
