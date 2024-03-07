package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Launcher extends SubsystemBase {
  private static Launcher instance;
  private double targetAngle;  // Target radians for the arm to be moved to
  private double targetLauncherVelocity;  // Velocity for rollers to spin at in rotations per second

  // Motor controllers
  private final TalonFX armMotor;  // Kraken for moving arm position
  private final TalonFX bottomLauncherMotor;  // Kraken for bottom roller
  private final TalonFX topLauncherMotor;  // Kraken for top roller
  private final WPI_TalonSRX feederMotor;  // Red motor

  private final MotionMagicExpoVoltage armMotionMagic;
  private final VelocityVoltage launcherController;

  private Launcher() {
    this.armMotor = new TalonFX(Constants.Ports.ARM_MOTOR, Constants.Ports.CTRE_CANBUS);
    this.bottomLauncherMotor = new TalonFX(Constants.Ports.BOTTOM_LAUNCHER, Constants.Ports.CTRE_CANBUS);
    this.topLauncherMotor = new TalonFX(Constants.Ports.TOP_LAUNCHER, Constants.Ports.CTRE_CANBUS);
    this.feederMotor = new WPI_TalonSRX(Constants.Ports.ARM_FEEDER);

    // Configure gains for launcher to be used in the controller
    Slot0Configs launcherGains = new Slot0Configs()
      .withKP(Constants.Arm.Launcher.P)
      .withKI(Constants.Arm.Launcher.I)
      .withKD(Constants.Arm.Launcher.D);

    // Limit current on launcher to avoid breaking motors
//    TorqueCurrentConfigs launcherCurrentConfigs = new TorqueCurrentConfigs()
//            .withPeakReverseTorqueCurrent(-Constants.Launcher.Launcher.MAX_CURRENT)
//            .withPeakForwardTorqueCurrent(Constants.Launcher.Launcher.MAX_CURRENT);

    CurrentLimitsConfigs launcherCurrentLimits = new CurrentLimitsConfigs().withStatorCurrentLimit(Constants.Arm.Launcher.MAX_CURRENT);

    // Apply configuration values and orientation
    TalonFXConfiguration topLauncherMotorConfig = new TalonFXConfiguration()
//            .withTorqueCurrent(launcherCurrentConfigs)
      .withCurrentLimits(launcherCurrentLimits)
      .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive))
      .withSlot0(launcherGains);
    this.topLauncherMotor.getConfigurator().apply(topLauncherMotorConfig);

    TalonFXConfiguration bottomLauncherMotorConfig = new TalonFXConfiguration()
//            .withTorqueCurrent(launcherCurrentConfigs)
      .withCurrentLimits(launcherCurrentLimits)
      .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive))
      .withSlot0(launcherGains);
    this.bottomLauncherMotor.getConfigurator().apply(bottomLauncherMotorConfig);

    // Configure gains to be used in a velocity controller
    this.launcherController = new VelocityVoltage(0.0)
      .withEnableFOC(true)
      .withSlot(0)
      .withUpdateFreqHz(1 / Constants.LOOP_INTERVAL);

//    TorqueCurrentConfigs armCurrentLimits = new TorqueCurrentConfigs()
//      .withPeakForwardTorqueCurrent(Constants.Launcher.Arm.MAX_CURRENT)
//      .withPeakReverseTorqueCurrent(-Constants.Launcher.Arm.MAX_CURRENT);
    FeedbackConfigs armFeedbackConfigs = new FeedbackConfigs()
      .withSensorToMechanismRatio(Constants.Arm.ArmMechanism.CONVERSION_FACTOR);

    Slot0Configs armMotorGains = new Slot0Configs()
      .withKP(Constants.Arm.ArmMechanism.P)
      .withKI(Constants.Arm.ArmMechanism.I)
      .withKD(Constants.Arm.ArmMechanism.D);

    TalonFXConfiguration armMotorConfig = new TalonFXConfiguration()
      .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive))
      .withCurrentLimits(new CurrentLimitsConfigs().withStatorCurrentLimit(Constants.Arm.ArmMechanism.MAX_CURRENT))
//      .withTorqueCurrent(armCurrentLimits)
      .withFeedback(armFeedbackConfigs)
      .withSlot0(armMotorGains);
    this.armMotor.getConfigurator().apply(armMotorConfig);

    // Configure MotionMagic Object
    this.armMotionMagic = new MotionMagicExpoVoltage(Constants.Arm.ArmMechanism.Positions.BASE)
      .withSlot(0)
      .withUpdateFreqHz(1 / Constants.LOOP_INTERVAL)
      .withEnableFOC(true);

    // Set Feeder Motor Limits
    this.feederMotor.setInverted(true);
    this.feederMotor.configContinuousCurrentLimit(Constants.Arm.Feeder.CONTINUOUS_CURRENT_LIMIT);
  }

  /**
    * Returns an initialized class of Launcher if one exists, or create a new one if it doesn't (and return it).
    * @return The launcher
   **/
  public static Launcher getInstance() {
    return Launcher.instance == null ? Launcher.instance = new Launcher() : Launcher.instance;
  }

  @Override
  public void periodic(){
    this.armMotor.setControl(this.armMotionMagic.withPosition(this.targetAngle));

//    this.topLauncherMotor.setControl(this.launcherController.withVelocity(this.targetLauncherVelocity));
//    this.bottomLauncherMotor.setControl(this.launcherController.withVelocity(this.targetLauncherVelocity));

    // if the launchers are at launching speed and the angle is reasonable, turn on feeder motor to give note to the launcher
//    if (launchersReady() && armReady()) {
//      this.feederMotor.setVoltage(0.0);
//    }
  }

  /**
    * Sets the target angle of the launcher.
    * @param angle The requested angle in radians.
   **/
  public void setTargetAngle(double angle) {
    this.targetAngle = angle;
  }

  /**
    * Sets the target velocity of the launcher motors.
    * @param velocity The requested velocity in radians per second.
   **/
  public void setTargetLauncherVelocity(double velocity) {
    this.targetLauncherVelocity = velocity;
  }

  /**
    * Get the rotational velocity of the top launcher motor.
    * @return The velocity of the motor in radians per second.
   **/
  private double getTopLauncherVelocity() {
    return this.topLauncherMotor.getVelocity().getValue() * 2 * Math.PI;
  }

  /**
    * Get the rotational velocity of the bottom launcher motor.
    * @return The velocity of the motor in radians per second.
   **/
  private double getBottomLauncherVelocity() {
    return this.bottomLauncherMotor.getVelocity().getValue() * 2 * Math.PI;
  }

  /**
    * Check if the velocity of the launcher motors are greater than or equal to the target velocity.
    * @return if the launchers are ready.
   **/
  private boolean launchersReady() {
    return (this.getTopLauncherVelocity() - this.targetLauncherVelocity) <= Constants.Arm.Launcher.VELOCITY_ERROR
      && (this.getBottomLauncherVelocity() - this.targetLauncherVelocity) <= Constants.Arm.Launcher.VELOCITY_ERROR;
  }

  /**
    * Check if the arm has reached its target angle.
    * @return A boolean representing if the arm is ready.
   **/
  private boolean armReady() {
    return Math.abs(this.armMotor.getPosition().getValueAsDouble() * 2 * Math.PI - this.targetAngle) <= Constants.Arm.ArmMechanism.POSITION_ERROR;
  }

  // Starts the feeder motor if it is ready
  public Command launchNote() {
    return this.run(() -> {
      if (launchersReady() && armReady()) {
        this.feederMotor.setVoltage(Constants.Arm.Feeder.TARGET_VOLTAGE);
      } else {
        this.feederMotor.setVoltage(0.0);
      }
    });
  }

  public void feederOff() {
    this.runOnce(() -> this.feederMotor.setVoltage(0.0));
  }

  // methods for each shot position for cleaner code in robot.java
  public void setIntake() {
    this.setTargetAngle(Constants.Arm.ArmMechanism.Positions.INTAKE);
    this.setTargetLauncherVelocity(Constants.Arm.Launcher.Velocities.INTAKE);
    this.feederMotor.setVoltage(Constants.Arm.Feeder.INTAKE_VOLTAGE);
  }

  public Command intakeNote() {
    return this.runOnce(() -> {
      this.feederMotor.setVoltage(Constants.Arm.Feeder.INTAKE_VOLTAGE);
//      this.setTargetLauncherVelocity(-20);
      this.bottomLauncherMotor.setControl(new VoltageOut(-2.0).withEnableFOC(true));
      this.topLauncherMotor.setControl(new VoltageOut(-2.0).withEnableFOC(true));
    });
  }
  public void setOverhandLaunch() {
    this.setTargetAngle(Constants.Arm.ArmMechanism.Positions.OVERHAND);
    this.setTargetLauncherVelocity(Constants.Arm.Launcher.Velocities.LAUNCH);
  }
  public void setUnderhandLaunch() {
    this.setTargetAngle(Constants.Arm.ArmMechanism.Positions.UNDERHAND);
    this.setTargetLauncherVelocity(Constants.Arm.Launcher.Velocities.LAUNCH);
  }
  public void setAmp() {
    this.setTargetAngle(Constants.Arm.ArmMechanism.Positions.AMP);
    this.setTargetLauncherVelocity(Constants.Arm.Launcher.Velocities.LAUNCH);
  }

  public void setBase() {
    this.setTargetAngle(Constants.Arm.ArmMechanism.Positions.BASE);
  }

  public void setLauncherVelocity(double speed) {
    this.targetLauncherVelocity = speed;
  }
}
