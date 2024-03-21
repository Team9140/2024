// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.revrobotics.CANSparkLowLevel;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.*;
import lib.util.REVSpark;
import org.littletonrobotics.junction.LoggedRobot;

public class Robot extends LoggedRobot {
  private Drivetrain drive;
  //  private PhotonVision camera;
  private Intake intake;
  private Arm arm;
  private Thrower thrower;
  private REVSpark climber;

  private Path path;

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
    this.path = Path.getInstance();
    NamedCommands.registerCommand("prepareLaunch", this.path.getPrepareOverhandLaunch());
    NamedCommands.registerCommand("launch", this.path.getOverhandLaunch());
    NamedCommands.registerCommand("intake", this.path.getIntakeOn());
    NamedCommands.registerCommand("intakeOff", this.path.getIntakeOff());
    Constants.UpdateSettings();

    // Silence verbose controller connection warnings
    DriverStation.silenceJoystickConnectionWarning(true);

//    this.camera = PhotonVision.getInstance();
    this.drive = Drivetrain.getInstance();
    this.intake = Intake.getInstance();
    this.arm = Arm.getInstance();
    this.thrower = Thrower.getInstance();
    this.climber = new REVSpark(Constants.Ports.CLIMBER, CANSparkLowLevel.MotorType.kBrushless);
    this.climber.setInverted(true);

    // Make the robot drive in Teleoperated mode by default
    this.drive.setDefaultCommand(Commands.run(() -> {
      // Remove low, fluctuating values from rotation input joystick
      double rightJoystickX = MathUtil.applyDeadband(this.controller.getHID().getRightX(), Constants.Drivetrain.TURN_DEADBAND);
      double leftJoystickY = MathUtil.applyDeadband(this.controller.getHID().getLeftY(), Constants.Drivetrain.DRIVE_DEADBAND);
      double leftJoystickX = MathUtil.applyDeadband(this.controller.getHID().getLeftX(), Constants.Drivetrain.DRIVE_DEADBAND);

      // Remove low, fluctuating values and drive at the input joystick as percentage of max velocity
      this.drive.swerveDrive(
        leftJoystickY * Constants.Drivetrain.METERS_PER_SECOND * -1,  // Forward (front-to-back) movement
        leftJoystickX * Constants.Drivetrain.METERS_PER_SECOND * -1,  // Horizontal (side-to-side) movement
        rightJoystickX * Math.abs(rightJoystickX) * Constants.Drivetrain.ROTATION_RADIANS_PER_SECOND * -1  // Rotation (squared to make larger values more sensitive)
      );
    }, this.drive));

    // FIXME: Find a way to not duplicate long command things
    // Prepare underhand throw
    this.controller.a().onTrue(this.arm.setUnderhand().alongWith(this.thrower.prepareSpeaker()).alongWith(this.intake.off()));

    // Prepare overhand throw
    this.controller.y().onTrue(this.arm.setOverhand().alongWith(this.thrower.prepareSpeaker()).alongWith(this.intake.off()));

    // Prepare amp throw
    this.controller.b().onTrue(this.arm.setAmp().alongWith(this.thrower.prepareAmp()).alongWith(this.intake.off()));

    // Stow TODO: Maybe not turn the intake off?
    this.controller.x().onTrue(this.arm.setStow().alongWith(this.intake.off()).alongWith(this.thrower.off()));

    // Eject everything
    this.controller.leftBumper()
      .onTrue(this.intake.reverseIntake().alongWith(this.thrower.setLauncherVoltage(12.0)).andThen(this.thrower.setFeederVoltage(12.0)))
      .onFalse(this.intake.off().alongWith(this.thrower.off()));

    // Intake Note
    this.controller.rightBumper()
      .onTrue(this.intake.intakeNote().alongWith(this.arm.setIntake().alongWith(this.thrower.setIntake())))
      .onFalse(this.intake.off().alongWith(this.arm.setStow()).alongWith(this.thrower.off()));

    // Throw note
    this.controller.rightTrigger()
      .onTrue(this.thrower.launch())
      .onFalse(new SequentialCommandGroup(
        this.thrower.launch(),
        new WaitCommand(0.25),
        this.arm.setStow().alongWith(this.thrower.off())
      ));

    // Toggle field-relative drive
    this.controller.start().onTrue(this.drive.toggleFieldRelative());

    // Reset gyro
    this.controller.back().onTrue(this.drive.resetGyro());

    this.addAutoModes();
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

    Command currentCommand = this.drive.getCurrentCommand();
    if (currentCommand != null) {
      SmartDashboard.putString("Auto Path", currentCommand.toString());
    } else {
      SmartDashboard.putString("Auto Path", "null");
    }
  }

  /**
   * Prepare autonomous mode.
   **/
  @Override
  public void autonomousInit() {
    Constants.UpdateSettings();
    CommandScheduler.getInstance().cancelAll();
    this.drive.resetPosition(Constants.STARTING_POSITION);
    this.path.auto().schedule();
  }

  /**
   * Prepare teleoperated mode.
   **/
  @Override
  public void teleopInit() {
//    CommandScheduler.getInstance().cancelAll();
    Constants.UpdateSettings();
  }

  private void addAutoModes() {
    Object[] startingPositions = Constants.STARTING_POSITIONS.keySet().toArray();

    for (int i = 0; i < startingPositions.length; i++) {
      if (i == Constants.DEFAULT_STARTING_POSITION) {
        Constants.positionChooser.setDefaultOption("[Position] " + startingPositions[i].toString() + " (Default)", i);
      } else {
        Constants.positionChooser.addOption("[Position] " + startingPositions[i].toString(), i);
      }
    }

    SmartDashboard.putData(Constants.positionChooser);
  }

  @Override
  public void simulationInit() {
    this.teleopInit();
  }

  @Override
  public void simulationPeriodic() {
    this.teleopPeriodic();
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }
}
