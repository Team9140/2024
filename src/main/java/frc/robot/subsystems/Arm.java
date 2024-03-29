package frc.robot.subsystems;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
  * The moving arm of the robot that is used to aim the launcher
 **/
public class Arm extends SubsystemBase {
  /**
    * Instance of the arm
   **/
  private static Arm instance;

  /**
    * Kraken motor that rotates the arm
   **/
  private final TalonFX motor = new TalonFX(Constants.Ports.ARM_MOTOR, Constants.Ports.CTRE_CANBUS);

  /**
    * A MagicMotion instance that outputs with a voltage. This is used to simplify PID control
   **/
  private final MotionMagicVoltage motionMagic;

  private Arm() {
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
    this.motor.setNeutralMode(NeutralModeValue.Brake);

    // Initialize Motion Magic
    this.motionMagic = new MotionMagicVoltage(Constants.Arm.Positions.INTAKE)
      .withEnableFOC(true)
      .withSlot(0)
      .withFeedForward(Constants.Arm.FEED_FORWARD)
      .withUpdateFreqHz(1 / Constants.LOOP_INTERVAL);

    // Set arm encoder position as starting if it is the first time booting the kraken
    if (Math.abs(this.getAngle()) < Constants.Arm.INITIAL_VARIANCE) this.motor.setPosition(-Math.PI / 2.0);
  }

  /**
    * Returns an initialized class of Arm if one exists, or create a new one if it doesn't (and return it).
    * @return The arm
   **/
  public static Arm getInstance() {
    return instance == null ? Arm.instance = new Arm() : Arm.instance;
  }

  /**
    * Routinely puts the arm position on the dashboard and enables motion magic motor control
   **/
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm Position", this.getAngle());
    this.motor.setControl(this.motionMagic);
  }


  /**
    * Checks if the arm is at the requested position
    * @return true if the arm is at the requested position, false otherwise
   **/
  public boolean isReady() {
    return Math.abs(this.getAngle() - this.motionMagic.Position) < Constants.Arm.AIM_ERROR;
  }

  /**
    * Get the arm position
   **/
  public double getAngle() {
    return this.motor.getPosition().getValueAsDouble();
  }

  /**
    * Arm goes to desired angle in radians using motionMagic
    * @return a command that sets the desired angle of the arm
   **/
  public Command setAngle(double position) {
    return this.runOnce(() -> this.motionMagic.withPosition(position));
  }

  /**
    * Moves arm to stowed position (which is the same as intake)
    * @return a command that sets the desired angle to the correct angle for intaking, aka the stow position
   **/
  public Command setStow() {
    return this.setIntake();
  }

  /**
    * Moves arm to intake position
    * @return a command that sets the desired angle to the angle required for intaking
   **/
  public Command setIntake() {
    return this.setAngle(Constants.Arm.Positions.INTAKE);
  }

  /**
    * Moves arm to overhand throwing position
    * @return a command that sets the desired angle to the angle required for overhead launching into the speaker
   **/
  public Command setOverhand() {
    return this.setAngle(Constants.Arm.Positions.OVERHAND);
  }

  /**
    * Moves arm to underhand throwing position
    * @return a command that sets the desired angle to the angle required for underhand launching into the speaker
   **/
  public Command setUnderhand() {
    return this.setAngle(Constants.Arm.Positions.UNDERHAND);
  }

  /**
    * Moves arm to throwing position for Amp
    * @return a command that sets the desired angle to the angle required for launching into the amp
   **/
  public Command setAmp() {
    return this.setAngle(Constants.Arm.Positions.AMP);
  }

  /**
    * Aim the launcher at the speaker and shoot a note
    * This function assumes you are angled correctly to shoot directly into the amp.
    * I can try to code a function that will do that for you, but ask the photonvision guys instead of me.
    * @param distance The distance of the speaker from the robot
   **/
  private void speakerAimBot(double distance) {
    double relativeHeight = Constants.Thrower.AutoAim.SPEAKER_HEIGHT - Constants.Thrower.AutoAim.JOINT_HEIGHT;

    double vVertical = Math.sqrt(
      Constants.Thrower.AutoAim.ENTERING_SPEAKER_VELOCITY * Constants.Thrower.AutoAim.ENTERING_SPEAKER_VELOCITY
      + 2 * Constants.Thrower.AutoAim.ACCELERATION_GRAVITY * relativeHeight
    );
    double vHorizontal = (distance * Constants.Thrower.AutoAim.ACCELERATION_GRAVITY) / (vVertical - Constants.Thrower.AutoAim.ENTERING_SPEAKER_VELOCITY);

    double theta = Math.asin(vVertical / Constants.Thrower.Launcher.LAUNCH_SPEED) - Constants.Thrower.AutoAim.ANGLE_ERROR_FIX;
    this.setAngle(theta);
  }
}
