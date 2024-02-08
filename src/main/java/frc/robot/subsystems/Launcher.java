package frc.robot.subsystems;

// importing phoenix6 stuff for talonfx controllers built into the calamari motors
import com.ctre.phoenix6.configs.TalonFXConfiguration; //object is a box of info that the controller uses
import com.ctre.phoenix6.controls.MotionMagicVoltage; //object is a request ??? 
import com.ctre.phoenix6.hardware.TalonFX; //object is the controller

// importing rev stuff for the little red motors
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel;

// importing wpilib stuff but idk how it works or what it really does
import edu.wpi.first.wpilibj2.command.Command; // CommandBase code is now moved into Command as of 2024
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Launcher extends SubsystemBase {

    private static Launcher instance; // single instance of Launcher class

    // motors (technically the motor controllers) 
    private final TalonFX armMotor; 
    // private final TalonFX bottomRollerMotor = new TalonFX(1,/* canbus? */); // kraken for bottom roller
    // private final TalonFX topRollerMotor = new TalonFX(2,/* canbus? */); // kraken for top roller
    // private final CANSparkMax feederMotor = new CANSparkMax(11, CANSparkLowLevel.MotorType.kBrushed); // little red motor

    // a request to motionmagic to move the arm to a position smoothly ???
    private final MotionMagicVoltage armMotionMagicVoltage;

    // constructor
    private Launcher(){
        // set up motors
        armMotor = new TalonFX(1); // other constructor TalonFX(1, string name of the CAN bus this device is on);
        
        // CONFIGURE TALONFXs
        TalonFXConfiguration configs = new TalonFXConfiguration(); // initialize a configuration to be set up
        // set .kP, .kI, .kD, .kV  for slot0 of configs
        // configs.Slot0.kP = ;
        // configs.Slot0.kI = ;
        // configs.Slot0.kD = ;
        // configs.Slot0.kV = ;
        armMotor.getConfigurator().apply(configs);// then write config to the talonfx

        // constructor: MotionMagicVoltage(double Position, boolean EnableFOC <- p2w, double FeedForward, int Slot, boolean boolean OverrideBrakeDurNeutral, boolean LimitForwardMotion, boolean LimitReverseMotion)
        armMotionMagicVoltage = new MotionMagicVoltage(0, false, 0, 0, false, false, false); 
    }

    public static Launcher getInstance(){
        return Launcher.instance == null ? Launcher.instance = new Launcher(): Launcher.instance;
    }

    @Override
    public void periodic(){
        // what is this supposed to do??? 
    }

    // eg: how to set launcher arm to a position called bruh
    // public Command setToPositionBruh(){
    //     return this.runOnce(() -> this.);
    // }


}
