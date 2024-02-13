package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Launcher extends SubsystemBase {

  private static Launcher instance; // single instance of Launcher class
  private double targetAngle; // position in radians for launcher to turn to
  private double targetShooterVelocity; // velocity in UNITS? for rollers to spin at

  // motors (technically the motor controllers)
  private TalonFX armMotor; // kraken for turning arm of launcher
  private TalonFX bottomShooterMotor; // kraken for bottom roller
  private TalonFX topShooterMotor; // kraken for top roller
  private CANSparkMax feederMotor; // little red motor

  // a request to motionmagic to move the arm to a position smoothly ???
  private MotionMagicExpoTorqueCurrentFOC armMotionMagic;

  // constructor
  private Launcher() {
    // initialize the motors
    this.armMotor = new TalonFX(5, Constants.Ports.LAUNCHER_CANBUS);
    this.bottomShooterMotor = new TalonFX(6,Constants.Ports.LAUNCHER_CANBUS);
    this.topShooterMotor = new TalonFX(7,Constants.Ports.LAUNCHER_CANBUS);
    this.feederMotor = new CANSparkMax(11, CANSparkLowLevel.MotorType.kBrushed);

    // configure talonfxs

    // TODO: armMotor needs conversion factor
    TalonFXConfiguration armMotorConfig = new TalonFXConfiguration();
    // armMotorConfig.Slot0.kP = ;
    // armMotorConfig.Slot0.kI = ;
    // armMotorConfig.Slot0.kD = ;
    // armMotorConfig.Slot0.kV = ;
    this.armMotor.getConfigurator().apply(armMotorConfig);

    TalonFXConfiguration shooterMotorConfig = new TalonFXConfiguration();
    this.topShooterMotor.getConfigurator().apply(shooterMotorConfig);
    this.bottomShooterMotor.getConfigurator().apply(shooterMotorConfig);

    this.bottomShooterMotor.setInverted(true);

    // other constructor: MotionMagicExpoTorqueCurrentFOC​(double Position, double FeedForward, int Slot, boolean OverrideCoastDurNeutral, boolean LimitForwardMotion, boolean LimitReverseMotion)
    this.armMotionMagic = new MotionMagicExpoTorqueCurrentFOC(0);
    this.armMotionMagic.UpdateFreqHz = 1000.0;
  }

  public static Launcher getInstance(){
    return Launcher.instance == null ? Launcher.instance = new Launcher() : Launcher.instance;
  }

  @Override
  public void periodic() {
    this.armMotor.setControl(this.armMotionMagic.withPosition(this.targetAngle).withSlot(0));
//    this.topShooterMotor.setControl();  // FIXME: fix it
//    this.bottomShooterMotor.setControl();
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

//  public Command feedNoteToShooter() {
//    return this.runOnce(() -> /* idk */);
//  }
}
