package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Launcher extends SubsystemBase {
  private static Launcher instance;
  private double targetAngle;  // Target radians for the arm to be moved to
  private double targetShooterVelocity;  // Velocity for rollers to spin at FIXME: unknown units

  // Motor controllers
  private TalonFX armMotor;  // Kraken for moving arm position
  private TalonFX bottomShooterMotor;  // Kraken for bottom roller
  private TalonFX topShooterMotor;  // Kraken for top roller
  private CANSparkMax feederMotor;  // Red motor

  private MotionMagicExpoTorqueCurrentFOC armMotionMagic;

  private Launcher() {
    this.armMotor = new TalonFX(Constants.Ports.ARM_MOTOR, Constants.Ports.LAUNCHER_CANBUS);
    this.bottomShooterMotor = new TalonFX(Constants.Ports.BOTTOM_SHOOTER, Constants.Ports.LAUNCHER_CANBUS);
    this.topShooterMotor = new TalonFX(Constants.Ports.TOP_SHOOTER, Constants.Ports.LAUNCHER_CANBUS);
    this.feederMotor = new CANSparkMax(Constants.Ports.ARM_FEEDER, CANSparkLowLevel.MotorType.kBrushed);

    // TODO: armMotor needs conversion factor
    TalonFXConfiguration armTalonFXConfigs = new TalonFXConfiguration();
    // armTalonFXConfigs.Slot0.kP = ;
    // armTalonFXConfigs.Slot0.kI = ;
    // armTalonFXConfigs.Slot0.kD = ;
    // armTalonFXConfigs.Slot0.kV = ;
//    TalonFXConfiguration shooterTalonFXConfigs = new TalonFXConfiguration();  // FIXME: unused

    TalonFXConfiguration shooterMotorConfig = new TalonFXConfiguration();
    this.topShooterMotor.getConfigurator().apply(shooterMotorConfig);
    this.bottomShooterMotor.getConfigurator().apply(shooterMotorConfig);
    this.bottomShooterMotor.setInverted(true);

    FeedbackConfigs armFeedbackConfigs = new FeedbackConfigs();
    armFeedbackConfigs.withSensorToMechanismRatio(Constants.Launcher.ARM_CONVERSION_FACTOR);
//    armTalonFXConfigs.apply(armFeedbackConfigs);  // FIXME: what is this?
    this.armMotor.getConfigurator().apply(armTalonFXConfigs);


    // Add MotionMagic to smoothly move the arm rotation
    // other constructor: MotionMagicExpoTorqueCurrentFOC​(double Position, double FeedForward, int Slot, boolean OverrideCoastDurNeutral, boolean LimitForwardMotion, boolean LimitReverseMotion)
    this.armMotionMagic = new MotionMagicExpoTorqueCurrentFOC(0);
    this.armMotionMagic.UpdateFreqHz = 1000.0;
  }

  public static Launcher getInstance() {
    return Launcher.instance == null ? Launcher.instance = new Launcher() : Launcher.instance;
  }

  @Override
  public void periodic(){
    this.armMotor.setControl(this.armMotionMagic.withPosition(this.targetAngle).withSlot(0));
//    this.topShooterMotor.setControl();  // FIXME: fix it
//    this.bottomShooterMotor.setControl();

    // if the shooters are at shooting speed and the angle is reasonable, turn on feeder motor to give note to the launcher
    if (areShootersReadyToShoot() && isArmReadyToShoot()) {
      this.feederMotor.setVoltage(0.0);
    }
  }

  // move arm to a position in radians
  public Command setPosition(double angle) {
    return this.runOnce(() -> this.targetAngle = angle);
  }

  public Command grabNote() {
    return this.runOnce(() -> this.targetShooterVelocity = Constants.Launcher.Velocities.GRAB);
  }

  public Command shootNote() {
    return this.runOnce(() -> this.targetShooterVelocity = Constants.Launcher.Velocities.SHOOT);
  }

  // get the velocity in radians per second of the topShooterMotor TODO: configure things to work in radians instead of rotations
  private double getTopShooterVelocity() {
    return this.topShooterMotor.getVelocity().getValue();
  }

  // get the velocity in radians per second of the bottomShooterMotor
  private double getBottomShooterVelocity() {
    return this.bottomShooterMotor.getVelocity().getValue();
  }

  // could use renaming; calls velocity getters for shooters and checks if theyre >= the constant for suitable shooting velocity
  private boolean areShootersReadyToShoot() {
    return this.getTopShooterVelocity() >= Constants.Launcher.Velocities.TOP_READY_TO_SHOOT && this.getBottomShooterVelocity() >= Constants.Launcher.Velocities.BOTTOM_READY_TO_SHOOT;
  }

  // true if launcher arm is not in intake/grab position
  private boolean isArmReadyToShoot() {
    return this.targetAngle != Constants.Launcher.Positions.INTAKE;
  }

  // true if launcher arm is in intake/grab position
  private boolean isArmReadyToGrabNote() {
    return this.targetAngle == Constants.Launcher.Positions.INTAKE;
  }

  // go to intake position
  public Command setIntake() {
    return setPosition(Constants.Launcher.Positions.INTAKE);
  }
}
