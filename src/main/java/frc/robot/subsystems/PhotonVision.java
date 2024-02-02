package frc.robot.subsystems;

import edu.wpi.first.math.geometry.*;
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
  private PhotonPipelineResult latestResult = null;
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

  public Offset getAngleOffBigTarget(){
    Offset ret = null;
    Pose3d robotPose = getRobotPose();
    Transform3d transform;
    if(robotPose != null) {
      switch (Constants.alliance.orElse(DriverStation.Alliance.Red)) {
        case Red:
          Pose3d tagPoseRed = Constants.Camera.field.getTagPose(4).get();
          transform = new Transform3d(robotPose, tagPoseRed);
          ret = new Offset(Math.sqrt(Math.pow(transform.getX(), 2) + Math.pow(transform.getY(), 2)),
                  transform.getRotation().toRotation2d().getRadians());
          break;

        case Blue:
          Pose3d tagPoseBlue = Constants.Camera.field.getTagPose(7).get();
          transform = new Transform3d(robotPose, tagPoseBlue);
          ret = new Offset(Math.sqrt(Math.pow(transform.getX(), 2) + Math.pow(transform.getY(), 2)),
                  transform.getRotation().toRotation2d().getRadians());
          break;
      }
    }
    return ret;
  }

  @Override
  public void periodic() {
    latestResult = camera.getLatestResult();
    SmartDashboard.putString("Camera Results", String.valueOf(this.camera.getLatestResult().hasTargets()));
  }

  public Pose3d getRobotPose(){
    if(latestResult.hasTargets()) {
      return PhotonUtils.estimateFieldToRobotAprilTag(latestResult.getBestTarget().getBestCameraToTarget(),
              Constants.Camera.field.getTagPose(latestResult.getBestTarget().getFiducialId()).get(), Constants.Camera.cameraToRobot);
    }else{
      return null;
    }
  }

  public class Offset {
    private double distance;
    private double rotation;

    public Offset(double distance, double rotation){
      this.distance = distance;
      this.rotation = rotation;
    }

    public double getDistance() {
      return distance;
    }

    public double getRotation() {
      return rotation;
    }
  }
}
