package frc.robot.subsystems;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  private static Arm instance;

  private final TalonFX motor;

  private final MotionMagicVoltage motionMagic;

  private Arm() {
    // Initialize Kraken
    motor = new TalonFX(Constants.Ports.ARM_MOTOR, Constants.Ports.CTRE_CANBUS);

    // Set PID and SVA values
    Slot0Configs launcherGains = new Slot0Configs()
      .withKP(Constants.Arm.P)
      .withKI(Constants.Arm.I)
      .withKD(Constants.Arm.D)
      .withKS(Constants.Arm.S)
      .withKV(Constants.Arm.V)
      .withKA(Constants.Arm.A);

    // Ensure that the motor doesn't die with current limits
    CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs().withStatorCurrentLimit(Constants.Arm.MAX_CURRENT).withStatorCurrentLimitEnable(true);

    // Sets units to radians of the physical arm
    FeedbackConfigs feedbackConfigs = new FeedbackConfigs().withSensorToMechanismRatio(Constants.Arm.SENSOR_TO_MECHANISM_RATIO);

    // Set max speed / acceleration limits
    MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs()
      .withMotionMagicCruiseVelocity(Constants.Arm.CRUISE_VELOCITY)
      .withMotionMagicAcceleration(Constants.Arm.ACCELERATION);

    // Set motor direction
    MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive);

    // Apply other configs
    TalonFXConfiguration motorConfig = new TalonFXConfiguration()
      .withSlot0(launcherGains)
      .withCurrentLimits(currentLimitsConfigs)
      .withFeedback(feedbackConfigs)
      .withMotionMagic(motionMagicConfigs)
      .withMotorOutput(motorOutputConfigs);
    this.motor.getConfigurator().apply(motorConfig);

    // Initialize Motion Magic
    this.motionMagic = new MotionMagicVoltage(Constants.Arm.Positions.INTAKE)
      .withEnableFOC(true)
      .withSlot(0)
      .withFeedForward(Constants.Arm.FEED_FORWARD)
      .withUpdateFreqHz(1 / Constants.LOOP_INTERVAL);

    // Set arm encoder position as starting if it is the first time booting the kraken
    if (Math.abs(this.motor.getPosition().getValueAsDouble()) < Units.degreesToRadians(Constants.Arm.INITIAL_VARIANCE)) this.motor.setPosition(-Math.PI / 2.0);
  }

  public static Arm getInstance() {
    return instance == null ? Arm.instance = new Arm() : Arm.instance;
  }

  public boolean isReady() {
    return Math.abs(this.motor.getPosition().getValueAsDouble() - this.motionMagic.Position) < Constants.Arm.AIM_ERROR;
  }

  // Arm goes to desired angle in radians using motionMagic
  public Command setAngle(double position) {
    return this.run(() -> this.motor.setControl(this.motionMagic.withPosition(position)));
  }

  // Moves arm to stowed position (which is the same as intake)
  public Command setStow() {
    return setIntake();
  }

  // Moves arm to intake position
  public Command setIntake() {
    return setAngle(Constants.Arm.Positions.INTAKE);
  }

  // Moves arm to overhand throwing position
  public Command setOverhand() {
    return setAngle(Constants.Arm.Positions.OVERHAND);
  }

  // Moves arm to underhand throwing position
  public Command setUnderhand() {
    return setAngle(Constants.Arm.Positions.UNDERHAND);
  }

  // Moves arm to throwing position for Amp
  public Command setAmp() {
    return setAngle(Constants.Arm.Positions.AMP);
  }

  /**
   * Aim the launcher at the speaker and shoot a note
   * @param distance The distance of the speaker from the robot
   **/
//  private void speakerAimBot(double distance) {
//    double relativeHeight = Constants.Launcher.SPEAKER_HEIGHT - Constants.Launcher.JOINT_HEIGHT;
//
//    double vVertical = Math.sqrt(Constants.Launcher.ENTERING_SPEAKER_VELOCITY * Constants.Launcher.ENTERING_SPEAKER_VELOCITY
//      + 2 * Constants.Launcher.ACCELERATION_GRAVITY * relativeHeight);
//    double vHorizontal = (distance * Constants.Launcher.ACCELERATION_GRAVITY) / (vVertical - Constants.Launcher.ENTERING_SPEAKER_VELOCITY);
//
//    double vTotal = Math.sqrt(vVertical*vVertical + vHorizontal*vHorizontal) + Constants.Launcher.TERMINAL_VELOCITY_ACCOUNTING;
//
//    this.targetShooterVelocity = vTotal / (Constants.Launcher.SHOOTER_RADIUS * 2 * Math.PI);
//    this.targetAngle = Math.atan2(vVertical, vHorizontal);
//  }
}
