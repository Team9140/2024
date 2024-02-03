package lib.util;

import edu.wpi.first.math.geometry.Twist2d;

public class Util {
    public static final double EPSILON = 0.00000001;

    public static boolean epsilonEquals(double a, double b, double epsilon) {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }

    public static boolean epsilonEquals(double a, double b) {
        return epsilonEquals(a, b, Util.EPSILON);
    }

    public static boolean epsilonEquals(Twist2d a, Twist2d b, double epsilon) {
        return epsilonEquals(a.dx, b.dx, epsilon) && epsilonEquals(a.dy, b.dy, epsilon) && epsilonEquals(a.dtheta, b.dtheta, epsilon);
    }
}
