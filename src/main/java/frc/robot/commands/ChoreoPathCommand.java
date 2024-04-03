package frc.robot.commands;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoControlFunction;
import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class ChoreoPathCommand extends Command {
  private final ChoreoTrajectory trajectory;
  private final ChoreoControlFunction controller;
  private final Timer timer;
  private final Drivetrain drive = Drivetrain.getInstance();

  public ChoreoPathCommand(String name) {
    this.addRequirements(this.drive);
    this.trajectory = Choreo.getTrajectory(name);
    this.controller = Choreo.choreoSwerveController(
      new PIDController(12.0, 0.0, 0.6),
      new PIDController(12.0, 0.0, 0.6),
      new PIDController(5.0, 0.0, 0.1)
    );
    this.timer = new Timer();
  }

  @Override
  public void initialize() {
    this.timer.restart();
  }

  @Override
  public void execute() {
    this.drive.swerveDrive(this.controller.apply(this.drive.getPosition(), this.trajectory.sample(this.timer.get())));
  }

  @Override
  public void end(boolean interrupted) {
    this.drive.swerveDrive(interrupted ? new ChassisSpeeds() : this.trajectory.getFinalState().getChassisSpeeds());
  }

  @Override
  public boolean isFinished() {
    return this.timer.hasElapsed(this.trajectory.getTotalTime());
  }
}
