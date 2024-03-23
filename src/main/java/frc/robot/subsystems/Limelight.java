package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

import java.util.Optional;

public class Limelight extends SubsystemBase {
    private static Limelight instance;
    public SwerveDrivePoseEstimator swervePose;


    public static Limelight getInstance() {
        return Limelight.instance == null
                ? Limelight.instance = new Limelight()
                : Limelight.instance;
    }

    /**
     * Routinely sends debugging information to SmartDashboard
     */
    @Override
    public void periodic() {
        SmartDashboard.putString("Camera junk: ", LimelightHelpers.getBotPose2d_wpiBlue("limelight").toString());
        Pose2d pose = LimelightHelpers.getBotPose2d_wpiBlue("limelight");
        SmartDashboard.putString("Camera Results", "X: " + pose.getX() + " Y: " + pose.getY());
        SmartDashboard.putString("Distance", "" + Units.metersToFeet(distanceFromGoal(pose)));
//        Optional<EstimatedRobotPose> pose = getRobotPose();
//        pose.ifPresent(estimatedRobotPose -> SmartDashboard.putString("Pose", estimatedRobotPose.estimatedPose.toString()));
//        pose.ifPresent(estimatedRobotPose -> SmartDashboard.putString("Distance: ", "" + distanceFromGoal(estimatedRobotPose.estimatedPose.toPose2d())));
//        pose.ifPresent(estimatedRobotPose -> SmartDashboard.putString("Closest point: ", getClosestScoringPoint(estimatedRobotPose).toString()));
        SmartDashboard.putString("Relative angle: ", "" + (angleRelativeToGoal(pose) * (180 / Math.PI)));
        SmartDashboard.putString("Score angle: ", angleToScore(pose).toString());
        SmartDashboard.putString("Get to score: ", getToScoringPosition(pose).toString());


    }

    /**
     * Gets the distance from a goal based on the AprilTag data as viewed by the camera
     * @param pose The robot's current position FIXME: uncertain
     * @return The distance from a goal
     **/
    public double distanceFromGoal(Pose2d pose){
        return switch (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)) {
            case Red -> {
                Transform2d transRed = new Transform2d(Constants.Camera.field.getTagPose(4).get().toPose2d(), pose);
                yield Math.sqrt(Math.pow(transRed.getX(), 2) + Math.pow(transRed.getY(), 2));
            }
            case Blue -> {
                Transform2d transBlue = new Transform2d(Constants.Camera.field.getTagPose(7).get().toPose2d(), pose);
                yield Math.sqrt(Math.pow(transBlue.getX(), 2) + Math.pow(transBlue.getY(), 2));
            }
        };
    }

    /**
     * Gets a coordinate point representing the closest scoring goal.
     * @return A Pose3d containing that coordinate
     *
     * In inches rn will change to meters later
     **/
    public Pose2d getClosestScoringPoint(Pose2d pose) {
        double xPoint = 0;
        double yPoint = 0;
        Rotation2d currentRotation = pose.getRotation();
        if (distanceFromGoal(pose) <= Constants.cameraRange) {
            return switch (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)) {
                case Blue:
                    xPoint = Constants.scoringRange * Math.cos(angleRelativeToGoal(pose)) - Units.inchesToMeters(1.5);
                    yPoint = Constants.scoringRange * Math.sin(angleRelativeToGoal(pose)) + Units.inchesToMeters(218.42);
                    yield new Pose2d(xPoint, yPoint, currentRotation);
                case Red:
                    xPoint = Constants.scoringRange * Math.cos(angleRelativeToGoal(pose)) + Units.inchesToMeters(652.73);
                    yPoint = Constants.scoringRange * Math.sin(angleRelativeToGoal(pose)) + Units.inchesToMeters(218.42);
                    yield new Pose2d(xPoint, yPoint, currentRotation);
            };
        }
        return new Pose2d();
    }

    /**
     * Returns angle relative to the goal regardless of what way the robot is facing
     * @return The angle relative to the goal in positive radians from 0 to 2 PI.
     *
     **/
    public double angleRelativeToGoal(Pose2d pose) {
        double angle = 0;
        return switch (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)) {
            //Gives angle in positive radians
            case Red -> ((2 * Math.PI) - Math.asin((pose.getY() - Units.inchesToMeters(218.42)) / Math.sqrt(
                    Math.pow(pose.getY() - Units.inchesToMeters(218.42), 2) + Math.pow(pose.getX() - Units.inchesToMeters(652.73), 2)
            ))) % (2 * Math.PI);
            case Blue -> ((4 * Math.PI) + Math.asin((pose.getY() - Units.inchesToMeters(218.42)) / Math.sqrt(
                    Math.pow(pose.getY() - Units.inchesToMeters(218.42), 2) + Math.pow(pose.getX() + Units.inchesToMeters(1.5), 2)
            ))) % (2 * Math.PI);
        };
    }

    /**
     * Returns the angle that the robot needs to be facing to be lined up with the goal
     */
    public Rotation2d angleToScore(Pose2d pose) {
        double currentAngle = pose.getRotation().getRadians();
        if (pose.getRotation().getRadians() < 0) {
            currentAngle = pose.getRotation().getRadians() + (2 * Math.PI);
        }
        switch (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)) {
            case Red -> {
                if (((currentAngle - (angleRelativeToGoal(pose) - Math.PI)) % (2 * Math.PI) * -1) > Math.PI) {
                    return new Rotation2d(((currentAngle - (angleRelativeToGoal(pose) + Math.PI)) % (2 * Math.PI) * -1) - (2 * Math.PI));
                }
                return new Rotation2d((currentAngle - (angleRelativeToGoal(pose) - Math.PI)) % (2 * Math.PI) * -1);
            }
            case Blue -> {
                if (((currentAngle - (angleRelativeToGoal(pose) + Math.PI)) % (2 * Math.PI) * -1) > Math.PI) {
                    return new Rotation2d(((currentAngle - (angleRelativeToGoal(pose) + Math.PI)) % (2 * Math.PI) * -1) - (2 * Math.PI));
                }
                return new Rotation2d((currentAngle - (angleRelativeToGoal(pose) + Math.PI)) % (2 * Math.PI) * -1);
            }
        };
        return new Rotation2d();
    }

    /**
     * Returns a Transform2d with the distance needed to move(if at all) and the rotation needed to face the goal
     * Can be adjusted to be in certain scoring spots rather than FIXME: unclear
     * @return The Transform2d object
     **/
    public Transform2d getToScoringPosition(Pose2d pose) {
        // If in scoring range assuming that the range has a radius
        if (distanceFromGoal(pose) <= Constants.scoringRange) {
            // Return Transform2d staying in same place and just rotating to line up with goal
            return new Transform2d(new Translation2d(), angleToScore(pose));
        } else if (distanceFromGoal(pose) <= Constants.cameraRange) {
            // Hopefully returns a Transform2d that tells robot where to go and how much to rotate by
            return new Transform2d(
                    new Translation2d(getClosestScoringPoint(pose).getX() - pose.getX(),
                            getClosestScoringPoint(pose).getY() - pose.getY()),
                    angleToScore(pose)
            );
        }
        return new Transform2d();
    }
}
