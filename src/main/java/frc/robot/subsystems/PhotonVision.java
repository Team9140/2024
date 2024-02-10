package frc.robot.subsystems;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.proto.Photon;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;
import java.util.Optional;

public class PhotonVision extends SubsystemBase {
  private static PhotonVision instance;
  private PhotonCamera camera;
  private PhotonPipelineResult latestResult = null;
  PhotonPoseEstimator photonPose;

  public static PhotonVision getInstance() {
    return PhotonVision.instance == null
      ? PhotonVision.instance = new PhotonVision()
      : PhotonVision.instance;
  }

  public PhotonVision() {
    this.camera = new PhotonCamera(Constants.Ports.CAMERA);
    photonPose = new PhotonPoseEstimator(Constants.Camera.field,
            PhotonPoseEstimator.PoseStrategy.AVERAGE_BEST_TARGETS, camera, Constants.Camera.cameraToRobot);

  }

  @Override
  public void periodic() {

  }

  //Gets current pose and timestamp on field using PhotonPoseEstimator
  public Optional<EstimatedRobotPose> getRobotPose(){
    return photonPose.update();
  }


  //takes a pose3d and returns the distance from the goal.
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
  /*
  Just writing down stuff to think about
  Objectives
    -Give "instructions" to other subsystems to run (Give coordinates to move to and how to get there)
      -Can look for potentially better way to interact with other subsystems
    -Use camera to identify AprilTags and then run some code based on which tag
    -Specifically look for tags 7 and 8 to run code to get into scoring position and send instructions to other subsystems
    -Use tag 8 for scoring(8 is lined with in the middle of the speaker)
      -meaning if robot recognizes tag 7, use that to find position on field to line up with 8 and score
    -What about amp? Do we want to be able to adjust launcher to shoot into amp?
      -amp takes 2 notes for 10 second boost
        -score comparison
          -2 points for amp + 5 * number of notes in speaker during 10s period: number of points = 2 + 5x
          -2 * number of notes in speaker + 4 points for the same number of notes it would take for the amp = 2x + 4
          -Amp seems much better, especially when hit at the right time(when teammates are ready to score)
    -Do we want the robot to go to a specific spot and then shoot or be in a "scoring range"
     and adjust angle based on where it is on the field?
    -Create method for recognizing AprilTags in stage assembly, then give instructions for putting robot in right spot
     to get onto the chain
    -Create method for finding out where on field robot is using AprilTags
      -Needs to take into account
        -Angle that the robot is recognizing it from
        -How far robot is from the AprilTag
          -Could use getArea(), less % = further and just need to test to find out % to distance ratio
            -If using getArea(), getSkew() needs to be used
             says "All of the data above (except skew) is available when using AprilTags", so maybe not
              -Need to find %area to %skew to distance ratio
              -Need to figure out how to identify where the %area is
                -To left vs to right of center of vision is two very different positions
        -Left vs right side of the AprilTag
          -Can use angle
          -getSkew() would be best but might not work
          -Depends on how angle is decided
            -Left: 180<270 degrees for angle of opposite corner on bottom line of AprilTag 90<180 for top of AprilTag
            -Right: 270<360 degrees for angle of opposite corner on bottom line 0< 90 for top of AprilTag
            -Opposite quadrant when using corner closest to us
          -Can possibly use pythagorean theorem using the corners?
            -Create a triangle with calculated measurements
            -Use inverse trig functions to get the angles within the triangle, then 90 - calculated angle?
    -METHODS that are useful for solving obj above
      -estimateFieldToRobotAprilTag(Transform3d cameraToTarget, Pose3d fieldRelativeTagPose, Transform3d cameraToRobot)
       returns your robot’s Pose3d on the field using the pose of the AprilTag relative to the camera,
       pose of the AprilTag relative to the field, and the transform from the camera to the origin of the robot.
       ex. Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(),
           aprilTagFieldLayout.getTagPose(target.getFiducialId()), cameraToRobot);
      -
   */

}
