// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drivetrain;
import org.littletonrobotics.junction.LoggedRobot;

public class Robot extends LoggedRobot {
  private Drivetrain drive;
  private final CommandXboxController controller = new CommandXboxController(Constants.Ports.INPUT_CONTROLLER);

  public Robot() {
    super(Constants.LOOP_INTERVAL);
  }

  @Override
  public void robotInit() {
    Constants.UpdateSettings();
    this.drive = Drivetrain.getInstance();

    this.controller.start().onTrue(this.drive.resetGyro());  // Temporary

    this.drive.setDefaultCommand(Commands.run(() -> {
      double rightJoystickX = MathUtil.applyDeadband(this.controller.getRightX(), Constants.Drivetrain.TURN_DEADBAND);
      this.drive.swerveDrive(
        MathUtil.applyDeadband(this.controller.getLeftY(), Constants.Drivetrain.DRIVE_DEADBAND) * Constants.Drivetrain.FORWARD_METERS_PER_SECOND * -1,
        MathUtil.applyDeadband(this.controller.getLeftX(), Constants.Drivetrain.DRIVE_DEADBAND) * Constants.Drivetrain.HORIZONTAL_METERS_PER_SECOND * -1,
        rightJoystickX * Math.abs(rightJoystickX) * Constants.Drivetrain.ROTATION_RADIANS_PER_SECOND * -1,
        this.controller.leftBumper().getAsBoolean());  // Removed ! from beginning to disable field-centric driving without the button press (TEMPORARY)
    }, this.drive));
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    SmartDashboard.putString("** chassis speed", this.drive.getSpeed().toString());
    SmartDashboard.putString("** chassis position", this.drive.getPosition().toString());
  }

  @Override
  public void autonomousInit() {
    Constants.UpdateSettings();
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().schedule(new PathPlannerAuto("New Auto"));
  }

  @Override
  public void teleopInit() {
    Constants.UpdateSettings();
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }
}
