package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private static Intake instance;

  private final WPI_TalonSRX frontLeftMotor;  // Big Bad Wolf
  private final WPI_TalonSRX frontRightMotor;  // Big Bad Wolf
  private final WPI_TalonSRX backMotor;  // Three Little Piggies

  private Intake() {
    this.frontLeftMotor = new WPI_TalonSRX(Constants.Ports.FRONT_LEFT_INTAKE);
    this.frontLeftMotor.configContinuousCurrentLimit(Constants.FRONT_INTAKE_CURRENT_LIMIT);

    this.frontRightMotor = new WPI_TalonSRX(Constants.Ports.FRONT_RIGHT_INTAKE);
    this.frontRightMotor.configContinuousCurrentLimit(Constants.FRONT_INTAKE_CURRENT_LIMIT);
    this.frontRightMotor.setInverted(true);
    this.frontRightMotor.follow(this.frontLeftMotor);

    this.backMotor = new WPI_TalonSRX(Constants.Ports.BACK_INTAKE);
    this.backMotor.configContinuousCurrentLimit(Constants.BACK_INTAKE_CURRENT_LIMIT);
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
      this.frontLeftMotor.setVoltage(Constants.FRONT_INTAKE_NOTE_VOLTS);
      this.backMotor.setVoltage(Constants.BACK_INTAKE_NOTE_VOLTS);
    });
  }

  /**
    * Releases a note that has been grabbed
    * Intake -> ground
   **/
  public Command releaseNote() {
    return this.runOnce(() -> {
      this.frontLeftMotor.setVoltage(-Constants.FRONT_INTAKE_NOTE_VOLTS);
      this.backMotor.setVoltage(-Constants.BACK_INTAKE_NOTE_VOLTS);
    });
  }

  /**
    * Turns off the intake motors
   **/
  public Command off() {
    return this.run(() -> {
      this.frontLeftMotor.setVoltage(0.0);
      this.backMotor.setVoltage(0.0);
    });
  }
}
