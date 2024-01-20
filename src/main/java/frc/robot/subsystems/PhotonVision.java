package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

public class PhotonVision extends SubsystemBase {
  private static PhotonVision instance;
  private PhotonCamera camera;

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

  //private class VisionData {}
}
