package lib.util;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;

public class Util {
  public static final double EPSILON = 0.00000001;
  public static final Rotation2d PI = Rotation2d.fromRadians(Math.PI);

  private static final Twist2d identity = new Twist2d();

  public static Twist2d twist2dIdentity() {
        return identity;
    }

  public static boolean epsilonEquals(double a, double b, double epsilon) {
    return (a - epsilon <= b) && (a + epsilon >= b);
  }

  public static boolean epsilonEquals(double a, double b) {
        return epsilonEquals(a, b, Util.EPSILON);
    }

  public static double getSparkPosition(CANSparkMax motor, double lastPosition) {
    return motor.getLastError().equals(REVLibError.kOk) ? motor.getEncoder().getPosition() : lastPosition;
  }

  public static double getSparkVelocity(CANSparkMax motor, double lastVelocity) {
    return motor.getLastError().equals(REVLibError.kOk) ? motor.getEncoder().getPosition() : lastVelocity;
  }
}
