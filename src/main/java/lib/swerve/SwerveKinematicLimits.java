package lib.swerve;

public class SwerveKinematicLimits {
    public final double kMaxDriveVelocity; // m/s
    public final double kMaxDriveAcceleration; // m/s^2
    public final double kMaxSteeringVelocity; // rad/s

    public SwerveKinematicLimits(double kMaxDriveVelocity, double kMaxDriveAcceleration, double kMaxSteeringVelocity) {
        this.kMaxDriveVelocity = kMaxDriveVelocity;
        this.kMaxDriveAcceleration = kMaxDriveAcceleration;
        this.kMaxSteeringVelocity = kMaxSteeringVelocity;
    }
}