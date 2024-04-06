package frc.robot.commands;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoControlFunction;
import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Auto;
import frc.robot.subsystems.Drivetrain;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;

public class ChoreoPathCommand extends Command {
  private final ChoreoTrajectory trajectory;
  private final ChoreoControlFunction controller;
  private final Timer timer;
  private final Drivetrain drive = Drivetrain.getInstance();
  private HashMap<Double, String> eventMarkers = new HashMap<>();
  private Double timeout = null;

  public ChoreoPathCommand(String name) {
    System.out.println(Timer.getFPGATimestamp() + " add requirements");
    this.addRequirements(this.drive);
    System.out.println(Timer.getFPGATimestamp() + " get trajectory");
    this.trajectory = Auto.getChoreoTrajectory(name);
    System.out.println(Timer.getFPGATimestamp() + " get choreo controller");
    this.controller = Choreo.choreoSwerveController(
      new PIDController(12.0, 0.0, 0.6),
      new PIDController(12.0, 0.0, 0.6),
      new PIDController(5.0, 0.0, 0.1)
    );
    System.out.println(Timer.getFPGATimestamp() + " create timer");
    this.timer = new Timer();
  }

  public void addEventMarker(double timestamp, String commandName) {
    this.eventMarkers.put(timestamp, commandName);
  }

  @Override
  public void initialize() {
    this.timer.restart();
  }

  @Override
  public void execute() {
    this.drive.swerveDrive(this.controller.apply(this.drive.getPosition(), this.trajectory.sample(this.timer.get())));

    if (!this.eventMarkers.isEmpty()) {
      List<Double> markers = new ArrayList<>(this.eventMarkers.keySet());
      Collections.sort(markers);

      double nextTimestamp = markers.get(0);
      if (this.timeout != null && this.timer.get() - nextTimestamp > this.timeout) {
        this.eventMarkers.remove(nextTimestamp);
      } else if (this.timer.get() >= nextTimestamp) {
        Auto.NamedCommands.getCommand(this.eventMarkers.get(nextTimestamp)).schedule();
        this.eventMarkers.remove(nextTimestamp);
      }
    }
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
