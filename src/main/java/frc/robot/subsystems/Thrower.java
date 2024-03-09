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

    private final TalonFX topLauncher;
    private final TalonFX bottomLauncher;

    private final WPI_TalonSRX feeder;

    private final VoltageOut launcherController;

    private Thrower() {
        this.topLauncher = new TalonFX(Constants.Ports.TOP_LAUNCHER, Constants.Ports.CTRE_CANBUS);
        this.bottomLauncher = new TalonFX(Constants.Ports.BOTTOM_LAUNCHER, Constants.Ports.CTRE_CANBUS);
        this.feeder = new WPI_TalonSRX(Constants.Ports.THROWER_FEEDER);

//        Slot0Configs launcherGains = new Slot0Configs()
//                .withKP(Constants.Thrower.Launcher.P)
//                .withKI(Constants.Thrower.Launcher.I)
//                .withKD(Constants.Thrower.Launcher.D)
//                .withKS(Constants.Thrower.Launcher.S)
//                .withKV(Constants.Thrower.Launcher.V)
//                .withKA(Constants.Thrower.Launcher.A);

        CurrentLimitsConfigs launcherCurrentLimits = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Constants.Thrower.Launcher.MAX_CURRENT)
                .withStatorCurrentLimitEnable(true);

        TalonFXConfiguration launcherConfiguration = new TalonFXConfiguration()
//                .withSlot0(launcherGains)
                .withCurrentLimits(launcherCurrentLimits);
        this.topLauncher.getConfigurator().apply(launcherConfiguration.withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive)));
        this.bottomLauncher.getConfigurator().apply(launcherConfiguration.withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive)));

        this.launcherController = new VoltageOut(0.0)
                .withEnableFOC(true)
                .withUpdateFreqHz(1 / Constants.LOOP_INTERVAL);

        this.feeder.setInverted(true);
        this.feeder.configContinuousCurrentLimit(Constants.Thrower.Feeder.MAX_CURRENT);
    }

    public Thrower getInstance() {
        return Thrower.instance == null ? Thrower.instance = new Thrower() : Thrower.instance;
    }

    public Command setLauncherVoltage(double voltage) {
        return this.run(() -> {
            this.launcherController.withOutput(voltage);
            this.topLauncher.setControl(this.launcherController);
            this.bottomLauncher.setControl(this.launcherController);
        });
    }

    public Command setFeederVoltage(double voltage) {
        return this.run(() -> this.feeder.setVoltage(voltage));
    }

    public Command setIntake() {
        return setLauncherVoltage(Constants.Thrower.Launcher.INTAKE_VOLTAGE)
                .alongWith(setFeederVoltage(Constants.Thrower.Feeder.INTAKE_VOLTAGE));
    }

    public Command prepareSpeaker() {
        return setLauncherVoltage(Constants.Thrower.Launcher.SPEAKER_VOLTAGE)
                .alongWith(setFeederVoltage(Constants.Thrower.Feeder.PREPARE_VOLTAGE));
    }

    public Command prepareAmp() {
        return setLauncherVoltage(Constants.Thrower.Launcher.AMP_VOLTAGE)
                .alongWith(setFeederVoltage(Constants.Thrower.Feeder.PREPARE_VOLTAGE));
    }

    public Command launch() {
        return this.run(() -> {
            this.topLauncher.setControl(this.launcherController);
            this.bottomLauncher.setControl(this.launcherController);
        }).alongWith(setFeederVoltage(Constants.Thrower.Feeder.LAUNCH_VOLTAGE));
    }

    public Command off() {
        return setLauncherVoltage(0.0)
                .alongWith(setFeederVoltage(Constants.Thrower.Feeder.PREPARE_VOLTAGE));
    }
}
