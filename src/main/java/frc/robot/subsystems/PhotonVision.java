package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

public class PhotonVision extends SubsystemBase {
  private static PhotonVision instance;
  private PhotonCamera camera;
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
  }

  public void getDistanceFromBigTarget() {
    switch (Constants.alliance.orElse(DriverStation.Alliance.Red)) {
      case Red:
        break;

      case Blue:
        break;
    }
  }

  @Override
  public void periodic() {
      SmartDashboard.putString("Camera Results", String.valueOf(this.camera.getLatestResult().hasTargets()));
      VisionData data = getPosition2d();
      SmartDashboard.putString("Timestamp", String.valueOf(data.getTimestamp()));
      SmartDashboard.putString("Pose", data.getPose().toString());
      SmartDashboard.putString("Ambiguity", String.valueOf(data.getAmbiguity()));
      SmartDashboard.putString("Tag Number", String.valueOf(data.getTagNum()));
  }

  public VisionData getPosition2d(){
    PhotonPipelineResult result = this.camera.getLatestResult();
    double gyro = 0.0;  // Temporary

    if (result.hasTargets()) {
      this.lastTagNum = result.getBestTarget().getFiducialId();
      Pose3d pose3d = Constants.Camera.field.getTagPose(this.lastTagNum).get();
      this.lastTimestamp = result.getTimestampSeconds();

      this.lastPose = PhotonUtils.estimateFieldToRobot(Constants.Camera.CAMERA_HEIGHT_METERS, pose3d.getZ(), Constants.Camera.CAMERA_PITCH_RADS,
        0.0, pose3d.getRotation().toRotation2d(), new Rotation2d(gyro), pose3d.toPose2d(),
        Constants.Camera.cameraToRobot
      );

      this.lastAmbiguity = result.getBestTarget().getPoseAmbiguity();
    }

    return new VisionData(this.lastTimestamp, this.lastPose, this.lastAmbiguity, this.lastTagNum);
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
