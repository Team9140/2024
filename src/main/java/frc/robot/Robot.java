// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Thrower;

import java.util.List;
//import org.littletonrobotics.junction.LoggedRobot;

public class Robot extends TimedRobot {
  private Drivetrain drive;
//  private PhotonVision camera;
  private Intake intake;

  private Arm arm;

  private Thrower thrower;

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
    this.arm = Arm.getInstance();
    this.thrower = Thrower.getInstance();
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

    // FIXME: Find a way to not duplicate long command things

    // Prepare underhand throw
    this.controller.a().onTrue(this.arm.setUnderhand()
            .alongWith(this.thrower.prepareSpeaker())
            .alongWith(this.intake.off()));
    // Prepare overhand throw
    this.controller.y().onTrue(this.arm.setOverhand()
            .alongWith(this.thrower.prepareSpeaker())
            .alongWith(this.intake.off()));
    // Prepare amp throw
    this.controller.b().onTrue(this.arm.setAmp()
            .alongWith(this.thrower.prepareAmp())
            .alongWith(this.intake.off()));
    // Stow TODO: Maybe not turn the intake off?
    this.controller.x().onTrue(this.arm.setStow()
            .alongWith(this.intake.off())
            .alongWith(this.thrower.off()));

    // Intake Note
    this.controller.rightBumper()
            .onTrue(this.intake.intakeNote().alongWith(this.arm.setIntake().alongWith(this.thrower.setIntake())))
            .onFalse(this.intake.off().alongWith(this.arm.setStow()).alongWith(this.thrower.off()));

    // Throw note
    this.controller.rightTrigger().onTrue(this.thrower.launch())
            .onFalse(new SequentialCommandGroup(this.thrower.launch()).alongWith(new WaitCommand(1.0),
                    this.arm.setStow().alongWith(this.thrower.off())));
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
    // Configure autonomous routines
    AutoBuilder.configureHolonomic(
            drive::getPosition,
            drive::resetPosition,
            drive::getSpeed,
            drive::swerveDrive,
            new HolonomicPathFollowerConfig(
                    Constants.Drivetrain.METERS_PER_SECOND,
                    Units.inchesToMeters(Math.hypot(Constants.WIDTH, Constants.LENGTH) / 2),
                    new ReplanningConfig()
            ),
            () -> Constants.alliance.isPresent() && Constants.alliance.get() == DriverStation.Alliance.Red,
            drive
    );

    Command overhandLaunchCommand =
            this.arm.setOverhand() // Set overhand aim
                    .alongWith(this.thrower.prepareSpeaker()) // Start launchers before throwing
                    .andThen(new WaitUntilCommand(this.arm::isReady)) // Wait until the launchers are spinning fast enough
                    .andThen(this.thrower.launch()) // Launch
                    .andThen(this.thrower.setIntake().alongWith(this.arm.setIntake())); // Adjust intake along with arm

    Command floorIntake =
            new WaitUntilCommand(this.arm::isReady) // Wait until the launchers are spinning fast enough
                    .andThen(this.intake.intakeNote()); // Start floor intake

    NamedCommands.registerCommand("Intake", floorIntake);
    NamedCommands.registerCommand("Intake_Off", this.intake.off());
    NamedCommands.registerCommand("Shoot",overhandLaunchCommand);

    List<PathPlannerPath> pathGroup = PathPlannerAuto.getPathGroupFromAutoFile("Auto1");
    PathPlannerPath path1 = pathGroup.get(0);
    PathPlannerPath path2 = pathGroup.get(1);
    PathPlannerPath path3 = pathGroup.get(2);
    PathPlannerPath path4 = pathGroup.get(3);
    PathPlannerPath path5 = pathGroup.get(4);
    PathPlannerPath path6 = pathGroup.get(5);

    new SequentialCommandGroup(
            overhandLaunchCommand,
            AutoBuilder.followPath(path1),
            AutoBuilder.followPath(path2),
            AutoBuilder.followPath(path3),
            AutoBuilder.followPath(path4),
            AutoBuilder.followPath(path5),
            AutoBuilder.followPath(path6)
    );
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
