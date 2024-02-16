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
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.Drivetrain;
import org.littletonrobotics.junction.LoggedRobot;

public class Robot extends LoggedRobot {
  private PhotonVision camera;

  private Drivetrain drive;
  private final CommandXboxController controller = new CommandXboxController(Constants.Ports.INPUT_CONTROLLER);

  public Robot() {
    super(Constants.LOOP_INTERVAL);
  }

  @Override
  public void robotInit() {
    Constants.UpdateSettings();
    this.camera = PhotonVision.getInstance();
    this.drive = Drivetrain.getInstance();

    this.drive.setDefaultCommand(Commands.run(() -> {
      double rightJoystickX = MathUtil.applyDeadband(this.controller.getHID().getRightX(), Constants.Drivetrain.TURN_DEADBAND);
      this.drive.swerveDrive(
        MathUtil.applyDeadband(this.controller.getHID().getLeftY(), Constants.Drivetrain.DRIVE_DEADBAND) * Constants.Drivetrain.METERS_PER_SECOND * -1,
        MathUtil.applyDeadband(this.controller.getHID().getLeftX(), Constants.Drivetrain.DRIVE_DEADBAND) * Constants.Drivetrain.METERS_PER_SECOND * -1,
        rightJoystickX * Math.abs(rightJoystickX) * Constants.Drivetrain.ROTATION_RADIANS_PER_SECOND * -1,
        !controller.getHID().getLeftBumper());
    }, this.drive));
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    if (controller.getHID().getAButton()) this.drive.resetGyro();
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
