// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Thrower;

import java.util.function.Consumer;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

public class Robot extends TimedRobot {
  private Drivetrain drive;  // The drivetrain instance
  private Intake intake;  // The intake instance
  private Arm arm;  // The arm instance
  private Thrower thrower;  // The thrower instance
  private CANSparkMax climber;  // The climber motor
  private final Auto auto = new Auto();  // The autonomous class

//  private final Candle candle = new Candle();

  // The input Xbox controller
  private final CommandXboxController driverController = new CommandXboxController(Constants.Ports.DRIVER_CONTROLLER);
  private final CommandXboxController auxController = new CommandXboxController(Constants.Ports.AUXILIARY_CONTROLLER);

  private enum ClimberPosition {
    Start,
    MovingUp,
    Up,
    MovingDown,
    Down
  }

  private ClimberPosition climberPosition = ClimberPosition.Start;

  private SysIdRoutine sysIdRoutine;

  /**
    * Initialize the robot and prepare it for operation.
   **/
  @Override
  public void robotInit() {
    Globals.UpdateSettings();
    // Silence verbose controller connection warnings
    DriverStation.silenceJoystickConnectionWarning(true);
//    this.candle.setAnimation(Candle.AnimationTypes.Fire);

    this.drive = Drivetrain.getInstance();
    this.intake = Intake.getInstance();
    this.arm = Arm.getInstance();
    this.thrower = Thrower.getInstance();
    this.climber = new CANSparkMax(Constants.Ports.CLIMBER, CANSparkLowLevel.MotorType.kBrushless);
    this.climber.setInverted(true);
    this.climber.setIdleMode(CANSparkBase.IdleMode.kBrake);

//    this.candle.setAnimation(Candle.AnimationTypes.Larson);

    // Eject everything
    this.driverController.leftBumper()
      .onTrue(this.intake.reverseIntake().alongWith(this.thrower.setLauncherVoltage(12.0)).andThen(this.thrower.setFeederVoltage(12.0)))
      .onFalse(this.intake.off().alongWith(this.thrower.off()));

    // Reset gyro
    this.driverController.back().onTrue(this.drive.resetGyro());

    switch (Constants.SYSID_MODE) {
      default:
      case Teleop:
        // Make the robot drive in Teleoperated mode by default
        this.drive.setDefaultCommand(this.drive.swerveDrive(
          this.driverController.getHID()::getLeftY,
          this.driverController.getHID()::getLeftX,
          this.driverController.getHID()::getRightX
        ));

        // Prepare underhand throw
        this.driverController.a().onTrue(
          this.arm.setUnderhand()
            .alongWith(this.thrower.prepareSpeaker())
            .alongWith(this.intake.off())
            .alongWith(this.drive.setDriveMultipliers(
              Constants.Drivetrain.Limits.UNDERHAND_LIN,
              Constants.Drivetrain.Limits.UNDERHAND_ROT
            ))
        );

        // Prepare overhand throw
        this.driverController.y().onTrue(
          this.arm.setOverhand()
            .alongWith(this.thrower.prepareSpeaker())
            .alongWith(this.intake.off())
            .alongWith(this.drive.setDriveMultipliers(
              Constants.Drivetrain.Limits.OVERHAND_LIN,
              Constants.Drivetrain.Limits.OVERHAND_ROT
            ))
        );

        // Prepare amp throw
        this.driverController.b().onTrue(
          this.arm.setAmp()
            .alongWith(this.thrower.prepareAmp())
            .alongWith(this.intake.off())
            .alongWith(this.drive.setDriveMultipliers(
              Constants.Drivetrain.Limits.AMP_LIN,
              Constants.Drivetrain.Limits.AMP_ROT
            ))
        );

        // Stow TODO: Maybe not turn the intake off?
        this.driverController.x().onTrue(
          this.arm.setStow()
            .alongWith(this.intake.off())
            .alongWith(this.thrower.off())
            .alongWith(this.drive.setDriveMultipliers(
              Constants.Drivetrain.Limits.NORMAL_LIN,
              Constants.Drivetrain.Limits.NORMAL_ROT
            ))
        );

        // Intake Note
        this.driverController.rightBumper()
          .onTrue(
            this.intake.intakeNote()
            .alongWith(this.arm.setIntake())
            .alongWith(this.thrower.setIntake())
            .alongWith(this.drive.setDriveMultipliers(
              Constants.Drivetrain.Limits.INTAKE_LIN,
              Constants.Drivetrain.Limits.INTAKE_ROT
            ))
          )
          .onFalse(this.intake.off().alongWith(this.arm.setStow()).alongWith(this.thrower.off()));

        // Throw note
        this.driverController.rightTrigger()
          .onTrue(this.thrower.launch())
          .onFalse(new SequentialCommandGroup(
            this.thrower.launch(),
            new WaitCommand(0.25),
            this.arm.setStow().alongWith(this.thrower.off())
          ));

        // Toggle field-relative drive
        this.driverController.start().onTrue(this.drive.toggleFieldRelative());

        break;
      case RotationSysId:
        this.setSysIdRoutine(
          2 * Math.PI, 4, 2 * Math.PI, 15,
          this.drive::rotateSysId, this.drive::getRotationalVelocity, "radians", () -> Units.degreesToRadians(this.drive.getGyroAngle()), "radians", this.drive
        );
        break;
      case StraightSysId:
        this.setSysIdRoutine(
          6, 10, 3, 15,
          this.drive::straightSysId, this.drive::getLinearVelocity, "meters", this.drive::getLinearDistanceMeters, "radians", this.drive
        );
        break;
//      case ThrowerRollerSysId:
//        this.setSysIdRoutine(
//
//        )
    }

    if (Constants.SYSID_MODE != Constants.SYSID.Teleop) {
      SignalLogger.setPath("/media/sda1/");
      this.driverController.a().whileTrue(this.sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse)).onFalse(this.drive.swerveDrive(() -> 0.0, () -> 0.0, () -> 0.0));
      this.driverController.y().whileTrue(this.sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward)).onFalse(this.drive.swerveDrive(() -> 0.0, () -> 0.0, () -> 0.0));
      this.driverController.x().whileTrue(this.sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse)).onFalse(this.drive.swerveDrive(() -> 0.0, () -> 0.0, () -> 0.0));
      this.driverController.b().whileTrue(this.sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward)).onFalse(this.drive.swerveDrive(() -> 0.0, () -> 0.0, () -> 0.0));
    }

    this.auto.getSelectedAutoId();
    Globals.addStartingPositions();

    (new Trigger(() -> this.thrower.getFeederCurrent() >= Constants.INTAKE_NOTIFY_CURRENT && this.isTeleop()))
      .onTrue(new InstantCommand(() -> this.driverController.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0.6)))
      .onFalse(new InstantCommand(() -> this.driverController.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0.0)));
  }

  private void setSysIdRoutine(
    double rateMagnitude, double rateSeconds, double stepMagnitude, double timeoutSeconds,
    Consumer<Measure<Voltage>> sysIdFunction, Supplier<Double> velocitySupplier, String velocityUnits, Supplier<Double> positionSupplier, String positionUnits, Subsystem subsystem
  ) {
    this.sysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(Volts.of(rateMagnitude).per(Seconds.of(rateSeconds)), Volts.of(stepMagnitude), Seconds.of(timeoutSeconds), state -> {
        SignalLogger.writeString("state", state.toString());
        SmartDashboard.putString("state", state.toString());
      }),
      new SysIdRoutine.Mechanism(sysIdFunction, l -> {
        SignalLogger.writeDouble("velocity", velocitySupplier.get(), velocityUnits);
        SignalLogger.writeDouble("position", positionSupplier.get(), positionUnits);
      }, subsystem)
    );
  }

  /**
    * Routinely execute the currently scheduled command.
   **/
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    SmartDashboard.putString("** chassis speed", this.drive.getChassisSpeeds().toString());
    SmartDashboard.putString("** chassis position", this.drive.getPosition().toString());
    SmartDashboard.putNumber("Time", Timer.getFPGATimestamp());
  }

  @Override
  public void disabledInit() {
    this.driverController.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0.0);
    if (Constants.SYSID_MODE != Constants.SYSID.Teleop) SignalLogger.stop();
  }

  /**
    * Routinely update settings while disabled. This fixes the issue from 2023 where the autonomous chooser would not
    * properly update.
   **/
  @Override
  public void disabledPeriodic() {
    Globals.UpdateSettings();
    if (
      !DriverStation.isJoystickConnected(Constants.Ports.DRIVER_CONTROLLER)
      || Math.abs(this.arm.getAngle() + 1.57) < Constants.Arm.AIM_ERROR
    ) {
//      this.candle.setAnimation(Candle.AnimationTypes.Error);
    } else {
//      this.candle.setAnimation(Candle.AnimationTypes.Rainbow);
    }

    SmartDashboard.putString("Current Auto", this.auto.getAutoName(this.auto.getSelectedAutoId()));
  }

  /**
    * Prepare autonomous mode.
   **/
  @Override
  public void autonomousInit() {
    System.out.println(Timer.getFPGATimestamp() + " cancelling commands");
    CommandScheduler.getInstance().cancelAll();
    System.out.println(Timer.getFPGATimestamp() + " resetting position");
    this.drive.resetPosition(Globals.STARTING_POSITION);
    System.out.println(Timer.getFPGATimestamp() + " run auto");
    this.auto.getAutoCommand().schedule();

//    this.candle.setAnimation(Candle.AnimationTypes.Twinkle);
  }

  /**
    * Prepare teleoperated mode.
   **/
  @Override
  public void teleopInit() {
    Globals.UpdateSettings();
    if (Constants.SYSID_MODE != Constants.SYSID.Teleop) SignalLogger.start();
    if (Globals.alliance == DriverStation.Alliance.Red) {
      // Red alliance
//      this.candle.setColor(255,0,0);
    } else {
      // Blue alliance
//      this.candle.setColor(0,0,255);
    }
  }

  @Override
  public void teleopPeriodic() {
    switch (this.climberPosition) {
      case MovingUp:
        this.climber.set(Constants.Climber.UP_VELOCITY);
        if (this.climber.getEncoder().getPosition() >= Constants.Climber.UP_POSITION) {
          this.climberPosition = ClimberPosition.Up;
        }
        break;
      case MovingDown:
        this.climber.set(this.driverController.getLeftTriggerAxis());
        if (this.climber.getEncoder().getPosition() >= Constants.Climber.DOWN_POSITION) {
          this.climberPosition = ClimberPosition.Down;
        }
        break;
      case Start:
        this.climber.set(0.0);
        if (this.driverController.getHID().getXButton() && this.driverController.getHID().getLeftTriggerAxis() >= Constants.Climber.ERROR) {
          this.climberPosition = ClimberPosition.MovingUp;
        }
        break;
      case Up:
        this.climber.set(0.0);
        if (this.driverController.getHID().getXButton() && this.driverController.getHID().getLeftTriggerAxis() >= Constants.Climber.ERROR) {
          this.climberPosition = ClimberPosition.MovingDown;
        }
        break;
      case Down:
        this.climber.set(0.0);
        break;
    }
    SmartDashboard.putString("Climber Target", this.climberPosition.toString());

//    if (this.driverController.getHID().getXButton()) {
//      this.climber.set(this.driverController.getHID().getLeftTriggerAxis());
//    } else {
//      this.climber.set(0.0);
//    }
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }
}
