// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.*;
import org.littletonrobotics.junction.LoggedRobot;

public class Robot extends LoggedRobot {
  /**
    * The drivetrain instance
   **/
  private Drivetrain drive;
  /**
    * The intake instance
   **/
  private Intake intake;
  /**
    * The arm instance
   **/
  private Arm arm;
  /**
    * The thrower instance
   **/
  private Thrower thrower;
  /**
    * The climber motor
   **/
  private CANSparkMax climber;
  /**
    * The autonomous auto class
   **/
  private final Auto auto = new Auto();

  private final Candle candle = new Candle();

  // The input Xbox controller
  private final CommandXboxController controller = new CommandXboxController(Constants.Ports.INPUT_CONTROLLER);

  private enum ClimberPosition {
    Start,
    MovingUp,
    Up,
    MovingDown,
    Down
  }
  private ClimberPosition climberPosition = ClimberPosition.Start;

  public Robot() {
    super(Constants.LOOP_INTERVAL);
  }

  /**
    * Initialize the robot and prepare it for operation.
   **/
  @Override
  public void robotInit() {
    Constants.UpdateSettings();
    // Silence verbose controller connection warnings
    DriverStation.silenceJoystickConnectionWarning(true);

    this.candle.setAnimation(Candle.AnimationTypes.Fire);

//    this.camera = PhotonVision.getInstance();
    this.drive = Drivetrain.getInstance();
    this.intake = Intake.getInstance();
    this.arm = Arm.getInstance();
    this.thrower = Thrower.getInstance();
    this.climber = new CANSparkMax(Constants.Ports.CLIMBER, CANSparkLowLevel.MotorType.kBrushless);
    this.climber.setInverted(true);
    this.climber.setIdleMode(CANSparkBase.IdleMode.kBrake);
    //this.candleSystem = Candle.getInstance();

    this.candle.setAnimation(Candle.AnimationTypes.Larson);

    // Make the robot drive in Teleoperated mode by default
    this.drive.setDefaultCommand(Commands.run(() -> {
      // Remove low, fluctuating values from rotation input joystick
      double rightJoystickX = MathUtil.applyDeadband(this.controller.getHID().getRightX(), Constants.Drivetrain.TURN_DEADBAND);
      double leftJoystickY = MathUtil.applyDeadband(this.controller.getHID().getLeftY(), Constants.Drivetrain.DRIVE_DEADBAND);
      double leftJoystickX = MathUtil.applyDeadband(this.controller.getHID().getLeftX(), Constants.Drivetrain.DRIVE_DEADBAND);

      // Remove low, fluctuating values and drive at the input joystick as percentage of max velocity
      this.drive.swerveDrive(
        leftJoystickY * Constants.Drivetrain.LINEAR_VELOCITY * -1,  // Forward (front-to-back) movement
        leftJoystickX * Constants.Drivetrain.LINEAR_VELOCITY * -1,  // Horizontal (side-to-side) movement
        rightJoystickX * Math.abs(rightJoystickX) * Constants.Drivetrain.ROTATION_VELOCITY * -1  // Rotation (squared to make larger values more sensitive)
      );
    }, this.drive));

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

    this.auto.getSelectedAutoId();
    this.addStartingPositions();
  }

  /**
    * Routinely execute the currently scheduled command.
   **/
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    SmartDashboard.putString("** chassis speed", this.drive.getSpeed().toString());
    SmartDashboard.putString("** chassis position", this.drive.getPosition().toString());
  }

  /**
    * Routinely update settings while disabled. This fixes the issue from 2023 where the autonomous chooser would not
    * properly update.
   **/
  @Override
  public void disabledPeriodic() {
    Constants.UpdateSettings();
    if (
      !DriverStation.isJoystickConnected(Constants.Ports.INPUT_CONTROLLER)
      || Math.abs(this.arm.getAngle() + 1.57) < Constants.Arm.AIM_ERROR
      || Math.abs(this.drive.getGyroAngle() - Constants.STARTING_POSITION.getRotation().getDegrees()) < 20.0
    ) {
      this.candle.setAnimation(Candle.AnimationTypes.Error);
    } else {
      this.candle.setAnimation(Candle.AnimationTypes.Rainbow);
    }

    SmartDashboard.putString("Current Auto", this.auto.getAutoName(this.auto.getSelectedAutoId()));
  }

  /**
    * Prepare autonomous mode.
   **/
  @Override
  public void autonomousInit() {
    Constants.UpdateSettings();
    CommandScheduler.getInstance().cancelAll();
    this.drive.resetPosition(Constants.STARTING_POSITION);
    int autoChoice = this.auto.getSelectedAutoId();
    if (autoChoice < Constants.CHOREO_AUTOS_OFFSET) {
      (switch (autoChoice) {
        default:
          System.out.println("Error: autoChoice value " + autoChoice + " is invalid. Defaulting to 0.");
        case 0:
          yield new SequentialCommandGroup(
            this.arm.setOverhand().alongWith(this.thrower.prepareSpeaker()),
            new WaitCommand(1),
            this.thrower.launch(),
            this.thrower.off().alongWith(this.arm.setStow()),
            this.drive.goStraight(1, 2)
          );
        case 1:
          yield new SequentialCommandGroup(
            this.arm.setOverhand().alongWith(this.thrower.prepareSpeaker()),
            new WaitCommand(0.75),
            this.thrower.launch(),
            new WaitCommand(0.25),
            this.thrower.off().alongWith(this.arm.setStow()),
            new WaitCommand(0.25),
            this.auto.getPathPlannerPath("BlueAmpSideTriple"),
            new WaitCommand(0.5),
            this.arm.setStow()
          );
        case 2:
          yield new SequentialCommandGroup(
            this.arm.setOverhand().alongWith(this.thrower.prepareSpeaker()),
            new WaitCommand(1),
            this.thrower.launch(),
            new WaitCommand(0.25),
            this.intake.intakeNote().alongWith(this.arm.setIntake()).alongWith(this.thrower.setIntake()),
            new WaitCommand(1),
            this.drive.goStraight(1.0, Units.inchesToMeters(50)),
            new WaitCommand(0.25),
            this.drive.goStraight(-1.0, Units.inchesToMeters(60)),
            this.intake.off().alongWith(this.thrower.off()),
            this.arm.setOverhand().alongWith(this.thrower.prepareSpeaker()),
            new WaitCommand(1),
            this.thrower.launch()
          );
        case 3:
          yield new SequentialCommandGroup(
            this.arm.setOverhand().alongWith(this.thrower.prepareSpeaker()),
            new WaitCommand(1),
            this.thrower.launch(),
            new WaitCommand(0.25),
            this.intake.intakeNote().alongWith(this.arm.setIntake()).alongWith(this.thrower.setIntake()),
            new WaitCommand(1),
            this.drive.goStraight(1.0, 0.0, Units.inchesToMeters(50)),
            new WaitCommand(0.25),
            this.drive.goStraight(-1.0, 0.0, Units.inchesToMeters(60)),
            this.intake.off().alongWith(this.thrower.off()),
            this.arm.setOverhand().alongWith(this.thrower.prepareSpeaker()),
            new WaitCommand(1),
            this.thrower.launch(),
            new WaitCommand(0.25),

            // add a 3rd note driving diagonal to another one
            this.intake.intakeNote().alongWith(this.arm.setIntake()).alongWith(this.thrower.setIntake()),
            new WaitCommand(1),
            this.drive.goStraight(0.47, -0.57, Units.inchesToMeters(80)),
            new WaitCommand(0.25),
            this.drive.goStraight(-0.47, 0.57, Units.inchesToMeters(90)),
            this.intake.off().alongWith(this.thrower.off()),
            this.arm.setOverhand().alongWith(this.thrower.prepareSpeaker()),
            new WaitCommand(1),
            this.thrower.launch(),
            this.thrower.off().alongWith(this.arm.setStow())
          );
//        case 4:  // Disabled for now
//          yield this.auto.auto("osdifhsodihg").alongWith(new SequentialCommandGroup(
//            // Timed events go here
//          ));
      }).schedule();
    } else {
      this.auto.getAutoCommand(this.auto.getSelectedAutoId()).schedule();
    }

    this.candle.setAnimation(Candle.AnimationTypes.Twinkle);
  }

  /**
    * Prepare teleoperated mode.
   **/
  @Override
  public void teleopInit() {
    Constants.UpdateSettings();
    if (Constants.alliance.isPresent() && Constants.alliance.get().equals(DriverStation.Alliance.Red)) {
      // Red alliance
      this.candle.setColor(255,0,0);
    } else {
      // Blue alliance
      this.candle.setColor(0,0,255);
    }
  }

  @Override
  public void teleopPeriodic() {
//    switch (this.climberPosition) {
//      case MovingUp:
//        this.climber.set(Constants.Climber.UP_VELOCITY);
//        if (this.climber.getEncoder().getPosition() >= Constants.Climber.UP_POSITION) {
//          this.climberPosition = ClimberPosition.Up;
//        }
//        break;
//      case MovingDown:
//        this.climber.set(Constants.Climber.DOWN_VELOCITY);
//        if (this.climber.getEncoder().getPosition() >= Constants.Climber.DOWN_POSITION) {
//          this.climberPosition = ClimberPosition.Down;
//        }
//        break;
//      case Start:
//        this.climber.set(0.0);
//        if (this.controller.getHID().getXButton() && this.controller.getHID().getLeftTriggerAxis() >= Constants.Climber.ERROR) {
//          this.climberPosition = ClimberPosition.MovingUp;
//        }
//        break;
//      case Up:
//        this.climber.set(0.0);
//        if (this.controller.getHID().getXButton() && this.controller.getHID().getLeftTriggerAxis() >= Constants.Climber.ERROR) {
//          this.climberPosition = ClimberPosition.MovingDown;
//        }
//        break;
//      case Down:
//        this.climber.set(0.0);
//        break;
//    }
    SmartDashboard.putString("Climber Target", this.climberPosition.toString());
    SmartDashboard.putNumber("Climber angle", this.climber.getEncoder().getPosition());

    if (this.controller.getHID().getXButton()) this.climber.set(this.controller.getHID().getLeftTriggerAxis());
  }

  private void addStartingPositions() {
    Object[] startingPositions = Constants.STARTING_POSITIONS.keySet().toArray();

    for (int i = 0; i < startingPositions.length; i++) {
      if (i == Constants.DEFAULT_STARTING_POSITION) {
        Constants.positionChooser.setDefaultOption("[Position] " + startingPositions[i].toString() + " (Default)", i);
      } else {
        Constants.positionChooser.addOption("[Position] " + startingPositions[i].toString(), i);
      }
    }

    SmartDashboard.putData("Positions", Constants.positionChooser);
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
