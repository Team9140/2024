package lib.util;

import com.revrobotics.*;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

/**
  * Complete wrapper around the CANSparkMax in order to force it to retry
  * configuration upon any error. Commented code is valid and can be used
  * but is commented to remove any warnings/notes from IntelliJ.
 **/
public class REVSpark implements MotorController, AutoCloseable {
  private CANSparkMax spark;
  private final int port;
  private final CANSparkLowLevel.MotorType type;


  public static boolean isNotOk(REVLibError result) {
    return !result.equals(REVLibError.kOk);// || result.equals(REVLibError.kCANDisconnected);
  }


  public REVSpark(int port, CANSparkLowLevel.MotorType type) {
    this.port = port;
    this.type = type;

    this.reloadCANSpark();
  }

  public void reloadCANSpark() {
    if (this.spark != null) this.close();
    this.spark = new CANSparkMax(this.port, this.type);
    System.out.println(this.spark);
  }

  public CANSparkMax getSpark() {
    return this.spark;
  }

  public RelativeEncoder getEncoder() {
    RelativeEncoder encoder;
    do {
      encoder = this.spark.getEncoder();
    } while (isNotOk(this.spark.getLastError()));
    return new REVEncoder(encoder);
  }

//  public RelativeEncoder getEncoder(SparkRelativeEncoder.Type encoderType, int countsPerRev) {
//    RelativeEncoder encoder;
//    do {
//      encoder = this.spark.getEncoder(encoderType, countsPerRev);
//    } while (!isOk(this.spark.getLastError()));
//    return new REVEncoder(encoder);
//  }
//
//  public RelativeEncoder getAlternateEncoder(int countsPerRev) {
//    RelativeEncoder encoder;
//    do {
//      encoder = this.spark.getAlternateEncoder(countsPerRev);
//    } while (!isOk(this.spark.getLastError()));
//    return new REVEncoder(encoder);
//  }
//
//  public RelativeEncoder getAlternateEncoder(SparkMaxAlternateEncoder.Type encoderType, int countsPerRev) {
//    RelativeEncoder encoder;
//    do {
//      encoder = this.spark.getAlternateEncoder(encoderType, countsPerRev);
//    } while (!isOk(this.spark.getLastError()));
//    return new REVEncoder(encoder);
//  }

  public AbsoluteEncoder getAbsoluteEncoder(SparkAbsoluteEncoder.Type type) {
    AbsoluteEncoder encoder;
    do {
      encoder = this.spark.getAbsoluteEncoder(type);
    } while (isNotOk(this.spark.getLastError()));
    return new REVEncoder(encoder);
  }

//  public REVLibError getLastError() {
//    return this.spark.getLastError();
//  }

  public void set(double speed) {
    this.spark.set(speed);
  }

  public void setVoltage(double voltage) {
    this.spark.setVoltage(voltage);
  }

  public double get() {
    return this.spark.get();
  }

  public void setInverted(boolean isInverted) {
    this.spark.setInverted(isInverted);
  }

  public boolean getInverted() {
    return this.spark.getInverted();
  }

  public void disable() {
    this.spark.disable();
  }

  public void stopMotor() {
    this.spark.stopMotor();
  }

  public void close() {
    try {
      this.spark.close();
    } catch (Exception e) {
      System.out.println("Error closing CANSparkMax: " + e.getMessage());
    }
  }

//  public SparkAnalogSensor getAnalog(SparkAnalogSensor.Mode mode) {
//    return this.spark.getAnalog(mode);
//  }

  public void restoreFactoryDefaults() {
    this.spark.restoreFactoryDefaults();
  }

  public void burnFlash() {
    while (isNotOk(this.spark.burnFlash())) {
      System.out.println("Error burning flash for CANSparkMax");
    }
  }

  public SparkPIDController getPIDController() {
    return this.spark.getPIDController();
  }

  public void setSmartCurrentLimit(int limit) {
    while (isNotOk(this.spark.setSmartCurrentLimit(limit))) {
      System.out.println("Error setting CANSparkMax smart current limit to " + limit);
    }
  }

//  public void setSmartCurrentLimit(int stallLimit, int freeLimit) {
//    while (!isOk(this.spark.setSmartCurrentLimit(stallLimit, freeLimit))) {
//      System.out.println("Error setting CANSparkMax smart current limit to " + stallLimit + " stall limit and " + freeLimit + " free limit");
//    }
//  }
//
//  public void setSmartCurrentLimit(int stallLimit, int freeLimit, int limitRPM) {
//    while (!isOk(this.spark.setSmartCurrentLimit(stallLimit, freeLimit, limitRPM))) {
//      System.out.println("Error setting CANSparkMax smart current limit to " + stallLimit + " stall limit, " + freeLimit + " free limit, and " + limitRPM + " RPM limit");
//    }
//  }
//
//  public void setSecondaryCurrentLimit(double limit) {
//    while (!isOk(this.spark.setSecondaryCurrentLimit(limit))) {
//      System.out.println("Error setting CANSparkMax secondary current limit to " + limit);
//    }
//  }
//
//  public void setSecondaryCurrentLimit(double limit, int chopCycles) {
//    while (!isOk(this.spark.setSecondaryCurrentLimit(limit, chopCycles))) {
//      System.out.println("Error setting CANSparkMax secondary current limit to " + limit + " with " + chopCycles + " chop cycles");
//    }
//  }

  public double getOutputCurrent() {
    return this.spark.getOutputCurrent();
  }

//  public void setIdleMode(CANSparkBase.IdleMode mode) {
//    while (!isOk(this.spark.setIdleMode(mode))) {
//      System.out.println("Error setting CANSparkMax idle mode to " + mode.toString());
//    }
//  }

//  public CANSparkBase.IdleMode getIdleMode() {
//    return this.spark.getIdleMode();
//  }


  public static class REVEncoder implements AbsoluteEncoder, RelativeEncoder {
    private final MotorFeedbackSensor encoder;
    private final boolean isAbsolute;

    public REVEncoder(AbsoluteEncoder encoder) {
      this.encoder = encoder;
      this.isAbsolute = true;
    }

    public REVEncoder(RelativeEncoder encoder) {
      this.encoder = encoder;
      this.isAbsolute = false;
    }

    public AbsoluteEncoder getAbsolute() {
      return this.isAbsolute ? (AbsoluteEncoder) this.encoder : null;
    }

    public RelativeEncoder getRelative() {
      return this.isAbsolute ? null : (RelativeEncoder) this.encoder;
    }

    public double getPosition() {
      return this.isAbsolute ? ((AbsoluteEncoder) this.encoder).getPosition() : ((RelativeEncoder) this.encoder).getPosition();
    }

    public double getVelocity() {
      return this.isAbsolute ? ((AbsoluteEncoder) this.encoder).getVelocity() : ((RelativeEncoder) this.encoder).getVelocity();
    }

    public REVLibError setPositionConversionFactor(double factor) {
      while (isNotOk(this.isAbsolute ? ((AbsoluteEncoder) this.encoder).setPositionConversionFactor(factor) : ((RelativeEncoder) this.encoder).setPositionConversionFactor(factor))) {
        System.out.println("Error setting CANSparkMax encoder position conversion factor to " + factor);
      }
      return REVLibError.kOk;
    }

    public double getPositionConversionFactor() {
      return this.isAbsolute ? ((AbsoluteEncoder) this.encoder).getPositionConversionFactor() : ((RelativeEncoder) this.encoder).getPositionConversionFactor();
    }

    public REVLibError setVelocityConversionFactor(double factor) {
      while (isNotOk(this.isAbsolute ? ((AbsoluteEncoder) this.encoder).setVelocityConversionFactor(factor) : ((RelativeEncoder) this.encoder).setVelocityConversionFactor(factor))) {
        System.out.println("Error setting CANSparkMax encoder velocity conversion factor to " + factor);
      }
      return REVLibError.kOk;
    }

    public double getVelocityConversionFactor() {
      return this.isAbsolute ? ((AbsoluteEncoder) this.encoder).getVelocityConversionFactor() : ((RelativeEncoder) this.encoder).getVelocityConversionFactor();
    }

    public REVLibError setZeroOffset(double offset) {
      if (!this.isAbsolute) {
        return REVLibError.kNotImplemented;
      }
      while (isNotOk(((AbsoluteEncoder) this.encoder).setZeroOffset(offset))) {
        System.out.println("Error setting CANSparkMax encoder zero offset to " + offset);
      }
      return REVLibError.kOk;
    }

    public double getZeroOffset() {
      return this.isAbsolute ? ((AbsoluteEncoder) this.encoder).getZeroOffset() : 0.0;
    }

    public REVLibError setInverted(boolean inverted) {
      while (isNotOk(this.encoder.setInverted(inverted))) {
        System.out.println("Error setting CANSparkMax encoder inverted to " + inverted);
      }
      return REVLibError.kOk;
    }

    public boolean getInverted() {
      return this.encoder.getInverted();
    }

    public REVLibError setPosition(double position) {
      if (this.isAbsolute) {
        return REVLibError.kNotImplemented;
      }
      while (isNotOk(((RelativeEncoder) this.encoder).setPosition(position))) {
        System.out.println("Error setting CANSparkMax encoder position to " + position);
      }
      return REVLibError.kOk;
    }

    public REVLibError setAverageDepth(int averageDepth) {
      while (isNotOk(this.isAbsolute ? ((AbsoluteEncoder) this.encoder).setAverageDepth(averageDepth) : ((RelativeEncoder) this.encoder).setAverageDepth(averageDepth))) {
        System.out.println("Error setting CANSparkMax encoder average depth to " + averageDepth);
      }

      return REVLibError.kOk;
    }

    public int getAverageDepth() {
      return this.isAbsolute ? ((AbsoluteEncoder) this.encoder).getAverageDepth() : ((RelativeEncoder) this.encoder).getAverageDepth();
    }

    public REVLibError setMeasurementPeriod(int i) {
      if (this.isAbsolute) {
        return REVLibError.kNotImplemented;
      }
      while (isNotOk(((RelativeEncoder) this.encoder).setMeasurementPeriod(i))) {
        System.out.println("Error setting CANSparkMax encoder measurement period to " + i);
      }
      return REVLibError.kOk;
    }

    public int getMeasurementPeriod() {
      return this.isAbsolute ? 0 : ((RelativeEncoder) this.encoder).getMeasurementPeriod();
    }

    public int getCountsPerRevolution() {
      return this.isAbsolute ? 0 : ((RelativeEncoder) this.encoder).getCountsPerRevolution();
    }
  }
}
