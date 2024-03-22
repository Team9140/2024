package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
// import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Thrower extends SubsystemBase {
  private static Thrower instance;

  // Big rollers that launch notes
  private final TalonFX topLauncher;
  private final TalonFX bottomLauncher;

  // Small feeder motor that gives notes to launchers
  private final WPI_TalonSRX feeder;

  // Provides output for launchers
  private final VoltageOut topLauncherController;
  private final VoltageOut bottomLauncherController;

  private double feederVolts;

  private Thrower() {
      // Creates motors
    this.topLauncher = new TalonFX(Constants.Ports.TOP_LAUNCHER, Constants.Ports.CTRE_CANBUS);
    this.bottomLauncher = new TalonFX(Constants.Ports.BOTTOM_LAUNCHER, Constants.Ports.CTRE_CANBUS);
    this.feeder = new WPI_TalonSRX(Constants.Ports.THROWER_FEEDER);

//    Slot0Configs launcherGains = new Slot0Configs()
//      .withKP(Constants.Thrower.Launcher.P)
//      .withKI(Constants.Thrower.Launcher.I)
//      .withKD(Constants.Thrower.Launcher.D)
//      .withKS(Constants.Thrower.Launcher.S)
//      .withKV(Constants.Thrower.Launcher.V)
//      .withKA(Constants.Thrower.Launcher.A);

    // Avoid losing launcher motors by providing current limits
    CurrentLimitsConfigs launcherCurrentLimits = new CurrentLimitsConfigs().withStatorCurrentLimit(Constants.Thrower.Launcher.MAX_CURRENT).withStatorCurrentLimitEnable(true);

    // Apply current limits, and Slot0 in the future
    TalonFXConfiguration launcherConfiguration = new TalonFXConfiguration().withCurrentLimits(launcherCurrentLimits);
//    .withSlot0(launcherGains)

    this.topLauncher.getConfigurator().apply(launcherConfiguration.withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive)));
    this.bottomLauncher.getConfigurator().apply(launcherConfiguration.withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive)));

    // Configure the launcher controller to use extra 15 percent power
    this.topLauncherController = new VoltageOut(0.0).withEnableFOC(true).withUpdateFreqHz(1 / Constants.LOOP_INTERVAL);
    this.bottomLauncherController = new VoltageOut(0.0).withEnableFOC(true).withUpdateFreqHz(1 / Constants.LOOP_INTERVAL);

    // Invert feeder motor, and provide current limits so it doesn't die
    this.feeder.setInverted(true);
    this.feeder.configContinuousCurrentLimit(Constants.Thrower.Feeder.MAX_CURRENT);
  }

  public static Thrower getInstance() {
    return Thrower.instance == null ? Thrower.instance = new Thrower() : Thrower.instance;
  }

  @Override
  public void periodic() {
    this.topLauncher.setControl(this.topLauncherController);
    this.bottomLauncher.setControl(this.bottomLauncherController);
    this.feeder.setVoltage(this.feederVolts);
  }


  // Creates command that repeatedly sets the provided launcher voltage
  public Command setLauncherVoltage(double topVoltage, double bottomVoltage) {
    return this.runOnce(() -> {
      this.topLauncherController.withOutput(topVoltage);
      this.bottomLauncherController.withOutput(bottomVoltage);
    });
  }

  public Command setLauncherVoltage(double voltage) {
    return this.setLauncherVoltage(voltage, voltage);
  }

  // Creates command that repeatedly sets the provided feeder voltage
  public Command setFeederVoltage(double voltage) {
      return this.runOnce(() -> this.feederVolts = voltage);
  }

  // Prepares launcher and feeder for intaking a note
  public Command setIntake() {
    return this.setLauncherVoltage(Constants.Thrower.Launcher.INTAKE_VOLTAGE).andThen(this.setFeederVoltage(Constants.Thrower.Feeder.INTAKE_VOLTAGE));
  }

  // Prepares launcher and feeder for launching to speaker
  public Command prepareSpeaker() {
    return this.setLauncherVoltage(Constants.Thrower.Launcher.SPEAKER_VOLTAGE).andThen(this.setFeederVoltage(Constants.Thrower.Feeder.PREPARE_VOLTAGE));
  }

  // Spins up Launchers at suitable speed for Amp and holds Feeder in place
  public Command prepareAmp() {
      return this.setLauncherVoltage(Constants.Thrower.Launcher.TOP_AMP_VOLTAGE, Constants.Thrower.Launcher.BOTTOM_AMP_VOLTAGE);
  }

  // Pushes the note from the feeder to the Launchers
  public Command launch() {
      return this.setFeederVoltage(Constants.Thrower.Feeder.LAUNCH_VOLTAGE);
  }

  // Stops Launchers and Feeder
  public Command off() {
    // TODO: replace andThen with alongWith?
    return this.setLauncherVoltage(0.0).andThen(this.setFeederVoltage(Constants.Thrower.Feeder.PREPARE_VOLTAGE));
  }
}
