package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private static Intake instance;

  private final CANSparkMax frontMotor;  // Big Bad Wolf
  private final CANSparkMax backMotor;  // Three Little Piggies

  private final SimpleMotorFeedforward feedforward;

  private Intake() {
    this.frontMotor = new CANSparkMax(Constants.Ports.FRONT_INTAKE, CANSparkLowLevel.MotorType.kBrushed);
    this.frontMotor.setSmartCurrentLimit(Constants.Launcher.INTAKE_CURRENT_LIMIT);

    this.backMotor = new CANSparkMax(Constants.Ports.BACK_INTAKE, CANSparkLowLevel.MotorType.kBrushed);
    this.backMotor.setSmartCurrentLimit(Constants.Launcher.INTAKE_CURRENT_LIMIT);
    this.backMotor.setInverted(true);

    this.feedforward = new SimpleMotorFeedforward(Constants.Launcher.INTAKE_S, Constants.Launcher.INTAKE_V, Constants.Launcher.INTAKE_A);
  }

  /**
    * called by robot.java to make sure there's only ever 1 instance of Intake
    * @return returns itself
   **/
  public Intake getInstance() {
    return Intake.instance == null ? Intake.instance = new Intake() : Intake.instance;
  }

  /**
    * hippity hoppity, this note is my property
    * @return A command that picks up a note
   **/
  public Command intakeNote() {
    return this.runOnce(() -> {
      double voltage = this.feedforward.calculate(Constants.Launcher.INTAKE_NOTE_VELOCITY);
      this.frontMotor.setVoltage(voltage);
      this.frontMotor.setVoltage(voltage);
    });
  }

  /**
    * Releases a note that has been grabbed from the intake onto the ground
    * @return A command that releases the note from the intake
   **/
  public Command releaseNote() {
    return this.runOnce(() -> {
      double voltage = this.feedforward.calculate(Constants.Launcher.INTAKE_RELEASE_VELOCITY);
      this.frontMotor.setVoltage(voltage);
      this.backMotor.setVoltage(voltage);
    });
  }

  /**
    * Turns off the intake motors
    * @return a command that turns off intake motors
   **/
  public Command off() {
    return this.runOnce(() -> {
      this.frontMotor.setVoltage(0.0);
      this.backMotor.setVoltage(0.0);
    });
  }
}