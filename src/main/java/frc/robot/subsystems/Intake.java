package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private static Intake instance;

  private final CANSparkMax frontMotor;  // Big Bad Wolf
  private final CANSparkMax backMotor;  // Three Little Piggies

  private Intake() {
    this.frontMotor = new CANSparkMax(Constants.Ports.FRONT_INTAKE, CANSparkLowLevel.MotorType.kBrushed);
    this.frontMotor.setSmartCurrentLimit(Constants.Launcher.INTAKE_CURRENT_LIMIT);

    this.backMotor = new CANSparkMax(Constants.Ports.BACK_INTAKE, CANSparkLowLevel.MotorType.kBrushed);
    this.backMotor.setSmartCurrentLimit(Constants.Launcher.INTAKE_CURRENT_LIMIT);
    this.backMotor.setInverted(true);
  }

  /**
    * Returns an initialized class of Intake if one exists, or create a new one if it doesn't (and return it).
    * @return The intake
   **/
  public static Intake getInstance() {
    return Intake.instance == null ? Intake.instance = new Intake() : Intake.instance;
  }

  /**
    * hippity hoppity, this note is now my property
    * Ground -> intake
    * NOTE: should be used before you are at the note, to ensure you can just run over it and voila it is now yours
    * does not grab it, just turns on the motors
   **/
  public Command intakeNote() {
    return this.run(() -> {
      this.frontMotor.setVoltage(Constants.Launcher.INTAKE_NOTE_VOLTS);
      this.backMotor.setVoltage(Constants.Launcher.INTAKE_NOTE_VOLTS);
    });
  }

  /**
    * Releases a note that has been grabbed
    * Intake -> ground
   **/
  public Command releaseNote() {
    return this.runOnce(() -> {
      this.frontMotor.setVoltage(-Constants.Launcher.INTAKE_NOTE_VOLTS);
      this.backMotor.setVoltage(-Constants.Launcher.INTAKE_NOTE_VOLTS);
    });
  }

  /**
    * Turns off the intake motors
   **/
  public Command off() {
    return this.run(() -> {
      this.frontMotor.setVoltage(0.0);
      this.backMotor.setVoltage(0.0);
    });
  }
}
