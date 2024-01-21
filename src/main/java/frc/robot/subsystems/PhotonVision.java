package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.proto.Photon;
import org.photonvision.targeting.PhotonPipelineResult;

public class PhotonVision extends SubsystemBase {
  private static PhotonVision instance;
  private PhotonCamera camera;
  private double lastTimestamp = 0.0;
  private Pose2d lastPose = null;

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
  }

  public VisionData getPosition(){
    PhotonPipelineResult result = this.camera.getLatestResult();
    if(result.hasTargets()){
      return new VisionData(result.getTimestampSeconds(), result.)
    }else{
      return new VisionData(lastTimestamp, lastPose);
    }
  }

  private class VisionData {
    private double timestamp;
    private Pose2d pose;

    public VisionData(double time, Pose2d pose){
      this.timestamp = time;
      this.pose = pose;
    }

    public double getTimestamp(){
      return this.timestamp;
    }

    public Pose2d getPose(){
      return this.pose;
    }
  }
}
