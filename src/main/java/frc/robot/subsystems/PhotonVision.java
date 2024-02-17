package frc.robot.subsystems;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

import java.util.Optional;

public class PhotonVision extends SubsystemBase {
  private static PhotonVision instance;
  private PhotonCamera camera;
  private PhotonPipelineResult latestResult = null;
  PhotonPoseEstimator photonPose;
  private double lastTimestamp = 0.0;
  private Pose2d lastPose = null;
  private double lastAmbiguity = 0.0;

  private int lastTagNum = -1;

  public static PhotonVision getInstance() {
    return PhotonVision.instance == null
      ? PhotonVision.instance = new PhotonVision()
      : PhotonVision.instance;
  }

  public PhotonVision() {
    this.camera = new PhotonCamera(Constants.Ports.CAMERA);
    photonPose = new PhotonPoseEstimator(
      Constants.Camera.field,
      PhotonPoseEstimator.PoseStrategy.AVERAGE_BEST_TARGETS,
      camera,
      Constants.Camera.cameraToRobot
    );
  }

  @Override
  public void periodic() {
    SmartDashboard.putString("Camera Results", "X: " + getRobotPose().get().estimatedPose.getX() + " Y: " + getRobotPose().get().estimatedPose.getY());
  }

  // Gets current pose and timestamp on field using PhotonPoseEstimator
  public Optional<EstimatedRobotPose> getRobotPose(){
    return photonPose.update();
  }

  /**
    * Gets the distance from a goal based on the AprilTag data as viewed by the camera
    * @param pose The robot's current position FIXME: uncertain
    * @return The distance from a goal
   **/
  public double distanceFromGoal(Pose3d pose){
    return switch (DriverStation.getAlliance().orElse(DriverStation.Alliance.Red)) {
      case Red -> {
        Transform3d transRed = new Transform3d(Constants.Camera.field.getTagPose(7).get(), pose);
        yield Math.sqrt(Math.pow(transRed.getX(), 2) + Math.pow(transRed.getY(), 2));
      }
      case Blue -> {
        Transform3d transBlue = new Transform3d(Constants.Camera.field.getTagPose(4).get(), pose);
        yield Math.sqrt(Math.pow(transBlue.getX(), 2) + Math.pow(transBlue.getY(), 2));
      }
    };
  }

  /**
    * Returns the relative angle to a goal FIXME: uncertain
    * @param pose The robot's current position FIXME: uncertain
    * @return A Rotation2d object containing the angle
   **/
  public Rotation2d angleFromGoal(Pose3d pose){
    return switch (DriverStation.getAlliance().orElse(DriverStation.Alliance.Red)) {
      case Red -> pose.getRotation().toRotation2d().minus(new Rotation2d(180));
      case Blue -> pose.getRotation().toRotation2d();
    };
  }

  /**
    * Gets a coordinate point representing the closest scoring goal.
    * @return A Pose3d containing that coordinate
   **/
  public Pose2d getClosestScoringPoint() {
    double xPoint = 0;
    double yPoint = 0;
    switch (DriverStation.getAlliance().orElse(DriverStation.Alliance.Red)) {
      // Math will change when angleRelativeToGoal is finished
      case Blue:
        xPoint = Constants.scoringRange * Math.cos(angleRelativeToGoal()) * -1;
        yPoint = 218.42 - Constants.scoringRange * Math.sin(angleRelativeToGoal());
      case Red:
        xPoint = 652.73 - Constants.scoringRange * Math.cos(angleRelativeToGoal());
        yPoint = 218 - Constants.scoringRange * Math.sin(angleRelativeToGoal());
    }
    return new Pose2d(xPoint, yPoint, getRobotPose().get().estimatedPose.getRotation().toRotation2d());
  }

  public VisionData getPosition2d() {
    PhotonPipelineResult result = this.camera.getLatestResult();
    double gyro = 0.0;  // Temporary

    if (result.hasTargets()) {
      this.lastTagNum = result.getBestTarget().getFiducialId();
      Pose3d pose3d = Constants.Camera.field.getTagPose(this.lastTagNum).get();
      this.lastTimestamp = result.getTimestampSeconds();

      this.lastPose = PhotonUtils.estimateFieldToRobot(Constants.Camera.CAMERA_HEIGHT_METERS, pose3d.getZ(), Constants.Camera.CAMERA_PITCH_RADS,
        0.0, pose3d.getRotation().toRotation2d(), new Rotation2d(gyro), pose3d.toPose2d(),
        Constants.Camera.cameraToRobot  // FIXME: error. wants Transform2d, given Transform3d
      );

      this.lastAmbiguity = result.getBestTarget().getPoseAmbiguity();
    }

    return new VisionData(this.lastTimestamp, this.lastPose, this.lastAmbiguity, this.lastTagNum);
  }

  /**
    * Returns angle relative to the goal regardless of what way the robot is facing
    * @return The angle relative to the goal.
   **/
  public double angleRelativeToGoal() {
    return switch (DriverStation.getAlliance().orElse(DriverStation.Alliance.Red)) {
      case Red -> Math.asin((getRobotPose().get().estimatedPose.getY() - 218.42) / Math.sqrt(
          Math.pow(getRobotPose().get().estimatedPose.getY() - 218.42, 2) + Math.pow(getRobotPose().get().estimatedPose.getX() - 652.73, 2)
        ));
      case Blue -> Math.asin((getRobotPose().get().estimatedPose.getY() - 218.42) / Math.sqrt(
          Math.pow(getRobotPose().get().estimatedPose.getY() - 218.42, 2) + Math.pow(getRobotPose().get().estimatedPose.getX() + 1.5, 2)
        ));
    };
  }

  /**
    * Returns a Transform2d with the distance needed to move(if at all) and the rotation needed to face the goal
    * Can be adjusted to be in certain scoring spots rather than FIXME: unclear
    * @return The Transform2d object
   **/
  public Transform2d getToScoringPosition() {
    // If in scoring range assuming that the range has a radius
    if (distanceFromGoal(getRobotPose().get().estimatedPose) <= Constants.scoringRange) {
      // Return Transform2d staying in same place and just rotating to line up with goal
      return new Transform2d(new Translation2d(), Rotation2d.fromRadians(getRobotPose().get().estimatedPose.getRotation().getZ()));
    } else {
      // Hopefully returns a Transform2d that tells robot where to go and how much to rotate by
      return new Transform2d(
        new Translation2d(Math.abs(getClosestScoringPoint().getX() - getRobotPose().get().estimatedPose.getX()),
        Math.abs(getClosestScoringPoint().getY() - getRobotPose().get().estimatedPose.getY())),
        Rotation2d.fromRadians(getRobotPose().get().estimatedPose.getRotation().getZ())
      );
    }
  }


  /**
    * Contains the 2d location of the robot, when that information is from, and how 'good' of a view the robot had
    * of the april tag which could help to inform on the accuracy.
   **/
  private class VisionData {
    private final double timestamp;
    private final Pose2d pose;
    private final double ambiguity;

    private final int tagNum;

    public VisionData(double time, Pose2d pose, double ambiguity, int tagNum) {
      this.timestamp = time;
      this.pose = pose;
      this.ambiguity = ambiguity;
      this.tagNum = tagNum;
    }

    public double getTimestamp() {
      return this.timestamp;
    }

    public Pose2d getPose() {
      return this.pose;
    }

    public double getAmbiguity() {
      return this.ambiguity;
    }

    public double getTagNum() {
      return this.tagNum;
    }
  }
}
