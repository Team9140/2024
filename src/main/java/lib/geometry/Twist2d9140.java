package lib.geometry;

import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import lib.util.Util;

public class Twist2d9140 extends Twist2d {

    public Twist2d9140(ChassisSpeeds chassisSpeeds) {
        this(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond, chassisSpeeds.omegaRadiansPerSecond);
    }

    public Twist2d9140(double vxMetersPerSecond, double vyMetersPerSecond, double omegaRadiansPerSecond) {
        super(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond);
    }

    public Twist2d9140() {
        super();
    }

    public boolean epsilonEquals(Twist2d other, double epsilon) {
        Twist2d9140 b = (Twist2d9140) other;
        return Util.epsilonEquals(this.dx, b.dx, epsilon) && Util.epsilonEquals(this.dy, b.dy, epsilon) && Util.epsilonEquals(this.dtheta, b.dtheta, epsilon);
    }

    public boolean epsilonEquals(Twist2d9140 other, double epsilon) {
        return epsilonEquals((Twist2d) other, epsilon);
    }

    public static Twist2d9140 identity() {
        return new Twist2d9140();
    }
}
