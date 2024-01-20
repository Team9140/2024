package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

public class PhotonVision {
    private PhotonCamera camera = new PhotonCamera("ProblemCamera");

    //gets last result from camera.
    PhotonPipelineResult result = camera.getLatestResult();

    public void getDistanceFromTarget() {
        switch(Constants.alliance) {
            case DriverStation.Alliance.Red:

                break;

            case DriverStation.Alliance.Blue:

                break;

        }
    }


}
