package frc.robot.subsystems;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.proto.Photon;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

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

  @Override
  public void periodic() {
    latestResult = camera.getLatestResult();
    SmartDashboard.putString("Camera Results", String.valueOf(this.camera.getLatestResult().hasTargets()));
  }

  //Gets current pose on field using AprilTag
  public Pose3d getRobotPose(){
    if(latestResult.hasTargets()) {
      return PhotonUtils.estimateFieldToRobotAprilTag(latestResult.getBestTarget().getBestCameraToTarget(),
              Constants.Camera.field.getTagPose(latestResult.getBestTarget().getFiducialId()).get(), Constants.Camera.cameraToRobot);
    }else{
      return null;
    }
  }

  //Method to get to a scoring position if close
  //Returns Offset containing the distance between current position and position we decide is our scoring position
  //and the rotation needed to line up properly(I think)
  public Offset getToScore() {
    Offset ret = null;
    Transform3d transform;
    Pose3d robotPose = getRobotPose();
    int[] scoringtags = {3,4,7,8};
    //Whatever pose we need to be in to score
    Pose3d scoringPose;
    //If speaker we are scoring on is in view, get the current pose and then moves to the position suited to score
    for (int i: scoringtags) {
        if(Constants.Camera.field.getTagPose(latestResult.getBestTarget().getFiducialId()).get()
                .equals(Constants.Camera.field.getTagPose(scoringtags[i]).get())) {
          transform = new Transform3d(robotPose, scoringPose);
          ret = new Offset(Math.sqrt(Math.pow(transform.getX(), 2) + Math.pow(transform.getY(), 2)),
                  transform.getRotation().toRotation2d().getRadians());
          break;
      }
    }
    return ret;
  }

  /*
  returns a transform2d of how the robot has to move in order to line up with target.
  returns null if none of the scoring tags (3, 4, 7, 8) are in view.
   */
  public Transform2d lineUpWithTarget(){
    Transform2d trans;
    PhotonTrackedTarget result = latestResult.getBestTarget();
    switch(result.getFiducialId()){
      case 3:
        trans = offCenter(result, 3, 4);
        break;
      case 8:
        trans = offCenter(result, 8, 7);
        break;
      case 4, 7:
        trans = onCenter(result);
        break;
      default:
        return null;
    }
    return trans;
  }

  //composes the transform of camera to tag, to the tag at the center of the goal, in theory producing the transform
  //from the camera to the center of the goal.
  public Transform2d offCenter(PhotonTrackedTarget result, int tagID1, int tagID2){
    Transform2d ret = new Transform2d(result.getBestCameraToTarget().getX(),
            result.getBestCameraToTarget().getY(), Rotation2d.fromRadians(result.getYaw()));
    return ret.plus(new Transform2d(Constants.Camera.field.getTagPose(tagID1).get().toPose2d(),
            Constants.Camera.field.getTagPose(tagID2).get().toPose2d()));
  }

  //just produces the transform from the camera to the center of the goal.
  public Transform2d onCenter(PhotonTrackedTarget result){
    return new Transform2d(result.getBestCameraToTarget().getX(),
            result.getBestCameraToTarget().getY(), Rotation2d.fromRadians(result.getYaw()));
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
