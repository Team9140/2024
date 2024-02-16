package frc.robot.subsystems;

// import stuff
import edu.wpi.first.wpilibj2.command.Command; // CommandBase code is merged into Command as of 2024
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase{

    // single instance
    public static instance;

    // idk if were using trapezoid profile from last year or MotionMagic

    // motor


    // constructor
    private Climber(){
        // set up motors have fun
    }

    // called by robot.java; this makes sure theres only ever 1 instance of Intake
    public Climber getInstance(){
        return frc.robot.subsystems.Climber.instance == null ? frc.robot.subsystems.Climber.instance = new Climber() : frc.robot.subsystems.Climber.instance;
    }

    // hes getting bigger!!!
    public Command extendClimber(){
        return runOnce(() -> {
            // tell motor to go uppy
        })
    }

    // short
    public Command retractClimber(){
        return runOnce(() -> {
            // tell motor to go down
        })
    }

    // true if up, false if down
    public boolean isUp(){
        return ; // check if motor sensor is past/at a certain position in radians 
    }


}