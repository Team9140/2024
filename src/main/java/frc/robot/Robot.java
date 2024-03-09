// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import org.littletonrobotics.junction.LoggedRobot;

public class Robot extends LoggedRobot {
  private Drivetrain drive;
//  private PhotonVision camera;
  private Intake intake;

  private CANSparkMax climber;

//  private Candle candleSystem = new Candle();

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

    // Silence verbose controller connection warnings
    DriverStation.silenceJoystickConnectionWarning(true);

//    this.camera = PhotonVision.getInstance();
    this.drive = Drivetrain.getInstance();
    this.intake = Intake.getInstance();
    this.climber = new CANSparkMax(Constants.Ports.CLIMBER, CANSparkLowLevel.MotorType.kBrushless);
    this.climber.setInverted(true);

    // Make the robot drive in Teleoperated mode by default
    this.drive.setDefaultCommand(Commands.run(() -> {
      // Remove low, fluctuating values from rotation input joystick
      double rightJoystickX = MathUtil.applyDeadband(this.controller.getHID().getRightX(), Constants.Drivetrain.TURN_DEADBAND);
      double leftJoystickY = MathUtil.applyDeadband(this.controller.getHID().getLeftY(), Constants.Drivetrain.DRIVE_DEADBAND);
      double leftJoystickX = MathUtil.applyDeadband(this.controller.getHID().getLeftX(), Constants.Drivetrain.DRIVE_DEADBAND);

      // Apply movement booster
      rightJoystickX = Constants.Drivetrain.TURN_REGULAR_NOBOOST * rightJoystickX + (1 - Constants.Drivetrain.TURN_REGULAR_NOBOOST) * rightJoystickX * this.controller.getHID().getLeftTriggerAxis();
      leftJoystickY = Constants.Drivetrain.DRIVE_REGULAR_NOBOOST * leftJoystickY + (1 - Constants.Drivetrain.DRIVE_REGULAR_NOBOOST) * leftJoystickY * this.controller.getHID().getLeftTriggerAxis();
      leftJoystickX = Constants.Drivetrain.DRIVE_REGULAR_NOBOOST * leftJoystickX + (1 - Constants.Drivetrain.DRIVE_REGULAR_NOBOOST) * leftJoystickX * this.controller.getHID().getLeftTriggerAxis();


      // Remove low, fluctuating values and drive at the input joystick as percentage of max velocity
      this.drive.swerveDrive(
        leftJoystickY * Math.abs(leftJoystickY) * Constants.Drivetrain.METERS_PER_SECOND * -1,  // Forward (front-to-back) movement
        leftJoystickX * Math.abs(leftJoystickX) * Constants.Drivetrain.METERS_PER_SECOND * -1,  // Horizontal (side-to-side) movement
        rightJoystickX * Math.abs(rightJoystickX) * Constants.Drivetrain.ROTATION_RADIANS_PER_SECOND * -1  // Rotation (squared to make larger values more sensitive)
      );
    }, this.drive));


  }
  /**
    * Routinely execute the currently scheduled command.
   **/
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    if (this.controller.getHID().getAButton()) this.climber.set(this.controller.getHID().getLeftTriggerAxis());

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
//    CommandScheduler.getInstance().schedule(new PathPlannerAuto("New Auto"));
  }

  /**
    * Prepare teleoperated mode.
   **/
  @Override
  public void teleopInit() {
    Constants.UpdateSettings();
  }


  @Override
  public void simulationInit() {
    this.teleopInit();
  }

  @Override
  public void simulationPeriodic() {
    this.teleopPeriodic();
//    System.out.println("")
  }

//  @Override
//  public void testInit() {
//    CommandScheduler.getInstance().cancelAll();
//  }
}
