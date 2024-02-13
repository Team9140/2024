package frc.robot.subsystems;

// importing phoenix6 stuff for talonfx controllers built into the calamari motors
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

// importing rev stuff for the little red motors
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel;

// importing wpilib stuff but idk how it works or what it really does
import edu.wpi.first.wpilibj2.command.Command; // CommandBase code is now moved into Command as of 2024
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Launcher extends SubsystemBase {

    private static Launcher instance; // single instance of Launcher class
    private double targetAngle; // position in radians for launcher to turn to
    private double targetRollerVelocity; // velocity in UNITS? for rollers to spin at

    // motors (technically the motor controllers) 
    private TalonFX armMotor; // kraken for turning arm of launcher 
    private TalonFX bottomRollerMotor; // kraken for bottom roller
    private TalonFX topRollerMotor; // kraken for top roller
    private CANSparkMax feederMotor; // little red motor

    // a request to motionmagic to move the arm to a position smoothly ???
    private MotionMagicExpoTorqueCurrentFOC armMotionMagic;

    // constructor
    private Launcher(){
        
        // initialize the motors
        this.armMotor = new TalonFX(1,/* canbus */);
        this.bottomRollerMotor = new TalonFX(2,/* canbus */);
        this.topRollerMotor = new TalonFX(3,/* canbus */);
        this.feederMotor = new CANSparkMax(11, CANSparkLowLevel.MotorType.kBrushed);

        // configure talonfxs

        // TODO armMotor needs conversion factor 
        TalonFXConfiguration armMotorConfig = new TalonFXConfiguration(); // initialize a configuration for armMotor
        // set .kP, .kI, .kD, .kV  for slot0 of armMotorConfig. This is for PID i think
        // armMotorConfig.Slot0.kP = ;
        // armMotorConfig.Slot0.kI = ;
        // armMotorConfig.Slot0.kD = ;
        // armMotorConfig.Slot0.kV = ;
        this.armMotor.getConfigurator().apply(armMotorConfig);// then write config to the talonfx

        TalonFXConfiguration rollerMotorConfig = new TalonFXConfiguration(); // initialize a configuration for both roller krakens (bottomRollerMotor, topRollerMotor)
        // set .kP, .kI, .kD, .kV  for slot0 of rollerMotorConfig. This is for PID i think
        // rollerMotorConfig.Slot0.kP = ;
        // rollerMotorConfig.Slot0.kI = ;
        // rollerMotorConfig.Slot0.kD = ;
        // rollerMotorConfig.Slot0.kV = ;
        this.topRollerMotor.getConfigurator().apply(rollerMotorConfig);
        this.bottomRollerMotor.getConfigurator().apply(rollerMotorConfig);
        
        //invert bottomRollerMotor (could be either)
        this.bottomRollerMotor.setInverted(true);

        // other constructor: MotionMagicExpoTorqueCurrentFOC​(double Position, double FeedForward, int Slot, boolean OverrideCoastDurNeutral, boolean LimitForwardMotion, boolean LimitReverseMotion)
        this.armMotionMagic = new MotionMagicExpoTorqueCurrentFOC(0);
        this.armMotionMagic.UpdateFreqHz = 1000.0;

    }

    public static Launcher getInstance(){
        return Launcher.instance == null ? Launcher.instance = new Launcher() : Launcher.instance;
    }

    @Override
    public void periodic(){
        this.armMotor.setControl(this.armMotionMagic.withPosition(this.targetAngle).withSlot(0));
        this.topRollerMotor.setControl();
        this.bottomRollerMotor.setControl();
    }

    // move arm to a position in radians
    public Command setPosition(double rad){
        return this.runOnce(() -> this.targetAngle = rad);
    }

    public Command rollersGrabNote(){
        return this.runOnce(() -> this.targetRollerVelocity = Constants.Launcher.Velocities.GRAB);
    }

    public Command rollersShootNote(){
        return this.runOnce(() -> this.targetRollerVelocity = Constants.Launcher.Velocities.SHOOT);
    }

    // eg: move arm to a position in radians called bruh ("bruh" doesnt actually exist tho)
    public Command setBruh(){
        return setPosition(Constants.Launcher.Positions.BRUH);
    }
}
