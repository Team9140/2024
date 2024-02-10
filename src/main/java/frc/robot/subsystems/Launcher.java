package frc.robot.subsystems;

// importing phoenix6 stuff for talonfx controllers built into the calamari motors
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
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
    private double targetAngle;

    // motors (technically the motor controllers) 
    private TalonFX armMotor; // kraken for turning arm of launcher 
    private TalonFX bottomRollerMotor; // kraken for bottom roller
    private TalonFX topRollerMotor; // kraken for top roller
    private CANSparkMax feederMotor; // little red motor

    // a request to motionmagic to move the arm to a position smoothly ???
    private MotionMagicVoltage armMotionMagicVoltage;
    private MotionMagicVelocityVoltage rollerMotionMagicVelocityVoltage;

    // constructor
    private Launcher(){
        
        // initialize the motors
        this.armMotor = new TalonFX(1,/* canbus */);
        this.bottomRollerMotor = new TalonFX(2,/* canbus */);
        this.topRollerMotor = new TalonFX(3,/* canbus */);
        this.feederMotor = new CANSparkMax(11, CANSparkLowLevel.MotorType.kBrushed);

        // configure talonfxs
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

        // constructor: MotionMagicVoltage(double Position, boolean EnableFOC <- p2w, double FeedForward, int Slot, boolean boolean OverrideBrakeDurNeutral, boolean LimitForwardMotion, boolean LimitReverseMotion)
        this.armMotionMagicVoltage = new MotionMagicVoltage(0, false, 0, 0, false, false, false);
        // constructor: MotionMagicVelocityVoltage(double Velocity, double Acceleration, boolean EnableFOC, double FeedForward, int Slot, boolean OverrideBrakeDurNeutral, boolean LimitForwardMotion, boolean LimitReverseMotion)
        this.rollerMotionMagicVelocityVoltage = new MotionMagicVelocityVoltage(0.0, 0.0, false, 0.0, 0, false, false, false);
    }

    public static Launcher getInstance(){
        return Launcher.instance == null ? Launcher.instance = new Launcher() : Launcher.instance;
    }

    @Override
    public void periodic(){
        this.armMotor.setControl(this.armMotionMagicVoltage.withPosition(this.targetAngle).withSlot(0));
    }

    // move arm to a position in radians
    public Command setPosition(double rad){
        return this.runOnce(() -> this.targetAngle = rad);
    }

    public Command rollersGrab(){

    }

    public Command rollersShoot(){
        
    }

    // eg: move arm to a position in radians called bruh ("bruh" doesnt actually exist tho)
    public Command setBruh(){
        return setPosition(Constants.Launcher.Positions.BRUH);
    }
}
