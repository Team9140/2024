package frc.robot.subsystems;

// import stuff

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  private static Climber instance;

  // TODO: implement TrapezoidProfile or MotionMagic

  private Climber() {}

  /**
    * Returns an initialized class of Climber if one exists, or create a new one if it doesn't (and return it).
    * @return The climber
   **/
  public Climber getInstance() {
    return Climber.instance == null ? Climber.instance = new Climber() : Climber.instance;
  }

  // hes getting bigger!!!
  public Command extendClimber() {
    return runOnce(() -> {
      // tell motor to go uppy
    });
  }

  // short
  public Command retractClimber() {
    return runOnce(() -> {
      // tell motor to go down
    });
  }

  // true if up, false if down
  public boolean isUp() {
    return false; // check if motor sensor is past/at a certain position in radians
  }
}