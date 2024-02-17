// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.PhotonVision;
import org.littletonrobotics.junction.LoggedRobot;

public class Robot extends LoggedRobot {
  // The camera subsystem instance
  private PhotonVision camera;

  // The drivetrain subsystem instance
  private Drivetrain drive;

  // The input Xbox controller
  private final CommandXboxController controller = new CommandXboxController(Constants.Ports.INPUT_CONTROLLER);

  public Robot() {
    super(Constants.LOOP_INTERVAL);
  }

  /**
    * Initialize the robot and prepare it for operation
   **/
  @Override
  public void robotInit() {
    Constants.UpdateSettings();

    // Create/get subsystem instances
    this.camera = PhotonVision.getInstance();
    this.drive = Drivetrain.getInstance();

    // Make the robot drive in eleoperated mode by default
    this.drive.setDefaultCommand(Commands.run(() -> {
      // Remove low, fluctuating values from rotation input joystick
      double rightJoystickX = MathUtil.applyDeadband(this.controller.getHID().getRightX(), Constants.Drivetrain.TURN_DEADBAND);

      // Remove low, fluctuating values and drive at the percentage of max velocity
      this.drive.swerveDrive(
        // Forward (front-to-back) movement
        MathUtil.applyDeadband(this.controller.getHID().getLeftY(), Constants.Drivetrain.DRIVE_DEADBAND) * Constants.Drivetrain.METERS_PER_SECOND * -1,

        // Horizontal (side-to-side) movement
        MathUtil.applyDeadband(this.controller.getHID().getLeftX(), Constants.Drivetrain.DRIVE_DEADBAND) * Constants.Drivetrain.METERS_PER_SECOND * -1,

        // Rotation (squared to make larger values more sensitive)
        rightJoystickX * Math.abs(rightJoystickX) * Constants.Drivetrain.ROTATION_RADIANS_PER_SECOND * -1,

        // Enable field-relative driving by default
        !this.controller.getHID().getLeftBumper()
      );
    }, this.drive));

  }

  /**
    * Routinely execute the currently scheduled command.
   **/
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    if (controller.getHID().getAButton()) this.drive.resetGyro();

    // TODO: Replace this with a button that will auto-align against a target and then shoot the note
    if (controller.getHID().getBButton()) Commands.run(() -> this.drive.swerveDrive(new Pose2d(5.0, 5.0, Rotation2d.fromDegrees(180))));

    SmartDashboard.putString("** chassis speed", this.drive.getSpeed().toString());
    SmartDashboard.putString("** chassis position", this.drive.getPosition().toString());
  }

  /**
    * Prepare autonomous mode.
   **/
  @Override
  public void autonomousInit() {
    Constants.UpdateSettings();

    this.drive.resetPosition(Constants.STARTING_POSITIONS[Constants.alliance_position.getAsInt()]);

    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().schedule(new PathPlannerAuto("New Auto"));
  }

  /**
    * Prepare teleoperated mode.
   **/
  @Override
  public void teleopInit() {
    Constants.UpdateSettings();
  }

//  @Override
//  public void testInit() {
//    CommandScheduler.getInstance().cancelAll();
//  }
}
