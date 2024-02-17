package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class MoveCommand extends Command {
  // PIDControllers for each movement type
  private final PIDController forwardPID;
  private final PIDController horizontalPID;
  private final PIDController rotationPID;

  // The drivetrain instance
  private final Drivetrain drive;

  // X, Y, and θ values for the target & error
  private final double targetX;
  private final double targetY;
  private final double targetTheta;

  private final double errorX;
  private final double errorY;
  private final double errorTheta;

  /**
    * A command that will move the robot to a specified field-relative position
    * @param target The target field position
    * @param error A Pose2d containing the acceptable error values
   **/
  public MoveCommand(Pose2d target, Pose2d error) {
    this.forwardPID = new PIDController(Constants.MoveCommand.FORWARD_P, Constants.MoveCommand.FORWARD_I, Constants.MoveCommand.FORWARD_D);
    this.horizontalPID = new PIDController(Constants.MoveCommand.HORIZONTAL_P, Constants.MoveCommand.HORIZONTAL_I, Constants.MoveCommand.HORIZONTAL_D);
    this.rotationPID = new PIDController(Constants.MoveCommand.ROTATION_P, Constants.MoveCommand.ROTATION_I, Constants.MoveCommand.ROTATION_D);
    this.drive = Drivetrain.getInstance();

    this.targetX = target.getX();
    this.targetY = target.getY();
    this.targetTheta = getRadians(target.getRotation());

    this.errorX = Math.abs(error.getX());
    this.errorY = Math.abs(error.getY());
    this.errorTheta = Math.abs(getRadians(error.getRotation()));
  }

  /**
    * Routinely calculate the current PID output values and move the robot accordingly.
   **/
  @Override
  public void execute() {
    Pose2d current = this.drive.getPosition();
    this.drive.swerveDrive(
      this.forwardPID.calculate(current.getX(), this.targetX),
      this.horizontalPID.calculate(current.getY(), this.targetY),
      this.rotationPID.calculate(getRadians(current.getRotation()), this.targetTheta),
      true
    );
  }

  /**
    * Check to see if the robot has reached the target position.
    * @return A boolean representing if the target has been reached
   **/
  @Override
  public boolean isFinished() {
    // Subtract the target from the current position
    Transform2d diff = this.drive.getPosition().minus(new Pose2d(this.targetX, this.targetY, Rotation2d.fromRadians(this.targetTheta)));
    // Compare the difference to see if it is in an acceptable range (the error values)
    return Math.abs(diff.getX()) <= this.errorX
      && Math.abs(diff.getY()) <= this.errorY
      && Math.abs(getRadians(diff.getRotation())) <= this.errorTheta;
  }

  /**
    * Returns the value of a Rotation2d in radians.
    * @param rotation The input Rotation2d
    * @return The rotation in radians
   **/
  private static double getRadians(Rotation2d rotation) {
    // TODO: Fix rotation to work in both directions across 0/2PI barrier
    return rotation.getRadians() % (2 * Math.PI);
  }
}
