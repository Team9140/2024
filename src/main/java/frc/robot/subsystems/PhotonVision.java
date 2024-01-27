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
import org.photonvision.proto.Photon;
import org.photonvision.targeting.PhotonPipelineResult;

import java.util.Optional;

public class PhotonVision extends SubsystemBase {
  private static PhotonVision instance;
  private PhotonCamera camera;
  private double lastTimestamp = 0.0;
  private Pose2d lastPose = null;

  private int lastTagNum = -1;

  // gets last result from camera.
//  PhotonPipelineResult result = camera.getLatestResult();

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
      SmartDashboard.putString("Tag Number", String.valueOf(data.getTagNum()));
  }

  public VisionData getPosition2d(){
    PhotonPipelineResult result = this.camera.getLatestResult();
    //temp
    double gyro = 0.0;
    if(result.hasTargets()){
      lastTagNum = result.getBestTarget().getFiducialId();
      Pose3d pose3d = Constants.Camera.field.getTagPose(lastTagNum).get();
      lastTimestamp = result.getTimestampSeconds();
      lastPose = PhotonUtils.estimateFieldToRobot(Constants.Camera.CAMERA_HEIGHT_METERS, pose3d.getZ(), Constants.Camera.CAMERA_PITCH_RADS,
              0.0, pose3d.getRotation().toRotation2d(), new Rotation2d(gyro), pose3d.toPose2d(),
              Constants.Camera.cameraToRobot);
      lastAmbiguity = result.getBestTarget().getPoseAmbiguity();

      return new VisionData(lastTimestamp, lastPose, lastTagNum);

    }else{
      return new VisionData(lastTimestamp, lastPose, lastTagNum);
    }
  }

  //Contains the 2d location of the robot, when that information is from, and how 'good' of a view the robot had
  // of the april tag which could help to inform on the accuracy.
  private class VisionData {
    private double timestamp;
    private Pose2d pose;
    private int tagNum;

    public VisionData(double time, Pose2d pose, int tagNum){
      this.timestamp = time;
      this.pose = pose;
      this.tagNum = tagNum;
    }

    public double getTimestamp(){
      return this.timestamp;
    }

    public Pose2d getPose(){
      return this.pose;
    }

    public double getTagNum(){
      return this.tagNum;
    }
  }
}
