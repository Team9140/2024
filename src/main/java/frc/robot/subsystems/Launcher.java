package frc.robot.subsystems;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Launcher extends SubsystemBase {
  private static Launcher instance;
  private double targetAngle;  // Target radians for the arm to be moved to
  private double targetShooterVelocity;  // Velocity for rollers to spin at FIXME: unknown units

  private final Launcher launcher = Launcher.getInstance();

  // Motor controllers
  private final TalonFX armMotor;  // Kraken for moving arm position
  private final TalonFX bottomShooterMotor;  // Kraken for bottom roller
  private final TalonFX topShooterMotor;  // Kraken for top roller
  private final CANSparkMax feederMotor;  // Red motor

  private final MotionMagicExpoTorqueCurrentFOC armMotionMagic;
  // private final Motion TODO: which motion magic to use for shooters

  private Launcher() {
    this.armMotor = new TalonFX(Constants.Ports.ARM_MOTOR, Constants.Ports.CTRE_CANBUS);
    this.bottomShooterMotor = new TalonFX(Constants.Ports.BOTTOM_SHOOTER, Constants.Ports.CTRE_CANBUS);
    this.topShooterMotor = new TalonFX(Constants.Ports.TOP_SHOOTER, Constants.Ports.CTRE_CANBUS);
    this.feederMotor = new CANSparkMax(Constants.Ports.ARM_FEEDER, CANSparkLowLevel.MotorType.kBrushed);

    TalonFXConfiguration shooterMotorConfig = new TalonFXConfiguration();
    this.topShooterMotor.getConfigurator().apply(shooterMotorConfig);
    this.bottomShooterMotor.getConfigurator().apply(shooterMotorConfig);
    this.bottomShooterMotor.setInverted(true);

    FeedbackConfigs armFeedbackConfigs = new FeedbackConfigs();
    armFeedbackConfigs.withSensorToMechanismRatio(Constants.Launcher.ARM_CONVERSION_FACTOR);
    TalonFXConfiguration armMotorConfig = new TalonFXConfiguration().withFeedback(armFeedbackConfigs);
    // armMotorConfig.Slot0.kP = ;
    // armMotorConfig.Slot0.kI = ;
    // armMotorConfig.Slot0.kD = ;
    // armMotorConfig.Slot0.kV = ;
    this.armMotor.getConfigurator().apply(armMotorConfig);

    // Add MotionMagic to smoothly move the arm rotation
//     other constructor: MotionMagicExpoTorqueCurrentFOC​(double Position, double FeedForward, int Slot, boolean OverrideCoastDurNeutral, boolean LimitForwardMotion, boolean LimitReverseMotion)
    this.armMotionMagic = new MotionMagicExpoTorqueCurrentFOC(0);
    this.armMotionMagic.UpdateFreqHz = 1000.0;
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
    this.armMotor.setControl(this.armMotionMagic.withPosition(this.targetAngle).withSlot(0));
//    this.topShooterMotor.setVoltage(voltage);
//    this.bottomShooterMotor.setVoltage(voltage);

    // if the shooters are at shooting speed and the angle is reasonable, turn on feeder motor to give note to the launcher
//    if (shootersReady() && armReady()) {
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
  public void setTargetShooterVelocity(double velocity) {
    this.targetShooterVelocity = velocity;
  }

  /**
    * Get the rotational velocity of the top launcher motor.
    * @return The velocity of the motor in radians per second.
   **/
  private double getTopShooterVelocity() {
    return this.topShooterMotor.getVelocity().getValue() * 2 * Math.PI;
  }

  /**
    * Get the rotational velocity of the bottom launcher motor.
    * @return The velocity of the motor in radians per second.
   **/
  private double getBottomShooterVelocity() {
    return this.bottomShooterMotor.getVelocity().getValue() * 2 * Math.PI;
  }

  /**
    * Check if the velocity of the launcher motors are greater than or equal to the target velocity.
    * @return if the shooters are ready.
   **/
  private boolean shootersReady() {
    return (this.getTopShooterVelocity() - this.targetShooterVelocity) <= Constants.Launcher.VELOCITY_ERROR
      && (this.getBottomShooterVelocity() - this.targetShooterVelocity) <= Constants.Launcher.VELOCITY_ERROR;
  }

  /**
    * Check if the arm has reached its target angle.
    * @return A boolean representing if the arm is ready.
   **/
  private boolean armReady() {
    return Math.abs(this.armMotor.getPosition().getValueAsDouble() * 2 * Math.PI - this.targetAngle) <= Constants.Launcher.POSITION_ERROR;
  }

  /*
   *  TODO: methods for each shot position
   */
}
