package lib.util;

import com.revrobotics.*;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

public class SparkWrapper implements MotorController, AutoCloseable {
  private CANSparkMax spark;
  private final int port;
  private final CANSparkLowLevel.MotorType type;

  private double speed = 0.0;
  private boolean inverted = false;
  private CANSparkBase.IdleMode idleMode = null;

  public SparkWrapper(int port, CANSparkLowLevel.MotorType type) {
    this.port = port;
    this.type = type;


    this.reloadCANSpark();
  }

  public boolean reloadCANSpark() {
//    if (this.spark != null) {
//      try {
        this.spark = new CANSparkMax(this.port, this.type);
        System.out.println("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n======================================================================== AGDSOUHUIHDSAFHUOSDFGHUOSDGOHUIDGSOIHASGOIAFSIOGASFIOGSAOIGASIOGADIHOGSADJIOGASOIJGASIJOGDASIJOJIOGDA");
        System.out.println(this.spark);
        return true;
//      } catch (Exception e) {
//        System.out.println("CANSparkMax not found on port " + this.port);
//        this.spark = null;
//        return false;
//      }
//    }
//    return true;
  }

  public RelativeEncoder getEncoder() {
    return this.spark != null ? this.spark.getEncoder() : new SimulatedEncoder();
  }

  public RelativeEncoder getEncoder(SparkRelativeEncoder.Type encoderType, int countsPerRev) {
    return this.spark != null ? this.spark.getEncoder(encoderType, countsPerRev) : new SimulatedEncoder(countsPerRev);
  }

  public RelativeEncoder getAlternateEncoder(int countsPerRev) {
    return this.spark != null ? this.spark.getAlternateEncoder(countsPerRev) : new SimulatedEncoder(countsPerRev);
  }

  public RelativeEncoder getAlternateEncoder(SparkMaxAlternateEncoder.Type encoderType, int countsPerRev) {
    return this.spark != null ? this.spark.getAlternateEncoder(encoderType, countsPerRev) : new SimulatedEncoder(countsPerRev);
  }

  public AbsoluteEncoder getAbsoluteEncoder(SparkAbsoluteEncoder.Type type) {
    return this.spark != null ? this.spark.getAbsoluteEncoder(type) : new SimulatedEncoder();
  }

  public void set(double speed) {
    if (this.spark != null) {
      this.spark.set(speed);
    } else {
      this.speed = speed;
    }
  }

  public void setVoltage(double voltage) {
    if (this.spark != null) {
      this.spark.setVoltage(voltage);
    }
  }

  public double get() {
    return this.spark != null ? this.spark.get() : this.speed;
  }

  public void setInverted(boolean isInverted) {
    if (this.spark != null) {
      this.spark.setInverted(isInverted);
    } else {
      this.inverted = isInverted;
    }
  }

  public boolean getInverted() {
    return this.spark != null ? this.spark.getInverted() : this.inverted;
  }

  public void disable() {
    if (this.spark != null) {
      this.spark.disable();
    }
  }

  public void stopMotor() {
    if (this.spark != null) {
      this.spark.stopMotor();
    }
  }

  public void close() throws Exception {
    if (this.spark != null) {
      this.spark.close();
    }
  }

  public SparkAnalogSensor getAnalog(SparkAnalogSensor.Mode mode) {
    return this.spark != null ? this.spark.getAnalog(mode) : null;
  }

  public void restoreFactoryDefaults() {
    if (this.spark != null) {
      this.spark.restoreFactoryDefaults();
    }
  }

  public REVLibError burnFlash() {
    if (this.spark != null) {
      return this.spark.burnFlash();
    }
    return null;
  }

  public SparkPIDController getPIDController() {
    return this.spark != null ? this.spark.getPIDController() : null;
  }

  public REVLibError setSmartCurrentLimit(int limit) {
    return this.spark != null ? this.spark.setSmartCurrentLimit(limit) : null;
  }

  public REVLibError setSmartCurrentLimit(int stallLimit, int freeLimit) {
    return this.spark != null ? this.spark.setSmartCurrentLimit(stallLimit, freeLimit) : null;
  }

  public REVLibError setSmartCurrentLimit(int stallLimit, int freeLimit, int limitRPM) {
    return this.spark != null ? this.spark.setSmartCurrentLimit(stallLimit, freeLimit, limitRPM) : null;
  }

  public REVLibError setSecondaryCurrentLimit(double limit) {
    return this.spark != null ? this.spark.setSecondaryCurrentLimit(limit) : null;
  }

  public REVLibError setSecondaryCurrentLimit(double limit, int chopCycles) {
    return this.spark != null ? this.spark.setSecondaryCurrentLimit(limit, chopCycles) : null;
  }

  public double getOutputCurrent() {
    return this.spark != null ? this.spark.getOutputCurrent() : 0.0;
  }

  public REVLibError setIdleMode(CANSparkBase.IdleMode mode) {
    if (this.spark != null) {
      return this.spark.setIdleMode(mode);
    }
    this.idleMode = mode;
    return null;
  }

  public CANSparkBase.IdleMode getIdleMode() {
    return this.spark != null ? this.spark.getIdleMode() : this.idleMode;
  }

  private static class SimulatedEncoder implements AbsoluteEncoder, RelativeEncoder {
    private double position = 0.0;
    private double positionConversionFactor = 1.0;
    private double velocityConversionFactor = 1.0;
    private int averageDepth = 0;
    private boolean inverted = false;
    private int measurementPeriod = 0;
    private int countsPerRevolution = 0;
    private double zeroOffset = 0.0;

    public SimulatedEncoder() {}

    public SimulatedEncoder(int countsPerRevolution) {
      this.countsPerRevolution = countsPerRevolution;
    }


    public double getPosition() {
      return this.inverted ? -position : position;
    }

    public double getVelocity() {
      return 0.0;
    }

    public REVLibError setPosition(double position) {
      this.position = this.inverted ? -position : position;
      return null;
    }

    public REVLibError setPositionConversionFactor(double conversionFactor) {
      this.positionConversionFactor = conversionFactor;
      return null;
    }

    public double getPositionConversionFactor() {
      return this.positionConversionFactor;
    }

    public REVLibError setVelocityConversionFactor(double conversionFactor) {
      this.velocityConversionFactor = conversionFactor;
      return null;
    }

    public double getVelocityConversionFactor() {
      return this.velocityConversionFactor;
    }

    public REVLibError setAverageDepth(int averageDepth) {
      this.averageDepth = averageDepth;
      return null;
    }

    public int getAverageDepth() {
      return this.averageDepth;
    }

    public REVLibError setZeroOffset(double offset) {
      this.zeroOffset = offset;
      return null;
    }

    public double getZeroOffset() {
      return this.zeroOffset;
    }

    public REVLibError setMeasurementPeriod(int i) {
      this.measurementPeriod = i;
      return null;
    }

    public int getMeasurementPeriod() {
      return this.measurementPeriod;
    }

    public int getCountsPerRevolution() {
      return this.countsPerRevolution;
    }

    public REVLibError setInverted(boolean b) {
      this.inverted = b;
      return null;
    }

    public boolean getInverted() {
      return this.inverted;
    }
  }
}
