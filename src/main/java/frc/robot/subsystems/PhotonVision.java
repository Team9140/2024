package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

public class PhotonVision extends SubsystemBase {
    private PhotonCamera camera = new PhotonCamera("ProblemCamera");

    //gets last result from camera.
    PhotonPipelineResult result = camera.getLatestResult();

    public void getDistanceFromBigTarget() {
        switch(Constants.alliance.orElse(DriverStation.Alliance.Red)) {
            case Red:

            break;

            case Blue:

            break;

        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("Camera Results", String.valueOf(camera.getLatestResult().hasTargets()));
    }
}
