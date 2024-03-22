package frc.robot.subsystems;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

import java.util.Optional;

public class PhotonVision extends SubsystemBase {
  private static PhotonVision instance;
  private final PhotonCamera camera = new PhotonCamera(Constants.Ports.CAMERA);
  private PhotonPipelineResult latestResult = null;
  private PhotonPoseEstimator photonPose;

  public static PhotonVision getInstance() {
    return PhotonVision.instance == null
      ? PhotonVision.instance = new PhotonVision()
      : PhotonVision.instance;
  }

  public PhotonVision() {
    this.photonPose = new PhotonPoseEstimator(
      Constants.Camera.field,
      PhotonPoseEstimator.PoseStrategy.AVERAGE_BEST_TARGETS,
      this.camera,
      Constants.Camera.cameraToRobot
    );
  }

  @Override
  public void periodic() {
    String coordinates = "X: " + this.getRobotPose().get().estimatedPose.getX() + " Y: " + this.getRobotPose().get().estimatedPose.getY();
    SmartDashboard.putString("Camera Results", coordinates);
  }

  //Gets current pose and timestamp on field using PhotonPoseEstimator
  public Optional<EstimatedRobotPose> getRobotPose(){
    return this.photonPose.update();
  }


  //takes a pose3d and returns the distance from the goal.
  public double distanceFromGoal(Pose3d pose){
    return switch (DriverStation.getAlliance()) {
      case Red:
        Transform3d transRed = new Transform3d(Constants.Camera.field.getTagPose(7).get(), pose);
        yield Math.sqrt(Math.pow(transRed.getX(), 2) + Math.pow(transRed.getY(), 2));
      case Blue:
        Transform3d transBlue = new Transform3d(Constants.Camera.field.getTagPose(4).get(), pose);
        yield Math.sqrt(Math.pow(transBlue.getX(), 2) + Math.pow(transBlue.getY(), 2));
    };
  }

  public Rotation2d angleFromGoal(Pose3d pose){
    return switch (DriverStation.getAlliance()) {
      case Red:
        yield pose.getRotation().toRotation2d().minus(new Rotation2d(180));
      case Blue:
        yield pose.getRotation().toRotation2d();
    };
  }

  //Returns a Pose3d
  public Pose2d getClosestScoringPoint() {
    double xPoint = 0;
    double yPoint = 0;
    switch (DriverStation.getAlliance()) {
      //math will change when angleRelativeToGoal is finished
      case Blue:
        xPoint = Constants.scoringRange * Math.cos(angleRelativeToGoal());
        yPoint = Constants.scoringRange * Math.sin(angleRelativeToGoal()) + 218.42;
      case Red:
        xPoint = 652.73 + Constants.scoringRange * Math.cos(angleRelativeToGoal());
        yPoint = 218 + Constants.scoringRange * Math.sin(angleRelativeToGoal());
    }
    return new Pose2d(xPoint, yPoint, getRobotPose().get().estimatedPose.getRotation().toRotation2d());
  }

  //Returns angle relative to the goal regardless of what way the robot is facing
  public double angleRelativeToGoal() {
    return switch (DriverStation.getAlliance().get()) {
      case Red:
        if (getRobotPose().get().estimatedPose.getY() >= 218.42) {
          yield Math.asin((getRobotPose().get().estimatedPose.getY() - 218.42)
            / Math.sqrt(Math.pow(getRobotPose().get().estimatedPose.getY() - 218.42, 2)
            + Math.pow(getRobotPose().get().estimatedPose.getX() - 652.73, 2)));
        } else {
          yield 2 * Math.PI - Math.asin((getRobotPose().get().estimatedPose.getY() - 218.42)
            / Math.sqrt(Math.pow(getRobotPose().get().estimatedPose.getY() - 218.42, 2)
            + Math.pow(getRobotPose().get().estimatedPose.getX() - 652.73, 2)));
        }
      case Blue:
        if (getRobotPose().get().estimatedPose.getY() >= 218.42) {
          yield Math.asin((getRobotPose().get().estimatedPose.getY() - 218.42)
            / Math.sqrt(Math.pow(getRobotPose().get().estimatedPose.getY() - 218.42, 2)
            + Math.pow(getRobotPose().get().estimatedPose.getX() + 1.5, 2)));
        } else {
          yield 2 * Math.PI - Math.asin((getRobotPose().get().estimatedPose.getY() - 218.42)
            / Math.sqrt(Math.pow(getRobotPose().get().estimatedPose.getY() - 218.42, 2)
            + Math.pow(getRobotPose().get().estimatedPose.getX() + 1.5, 2)));
        }
    };
  }

  // Returns a Transform2d with the distance needed to move(if at all) and the rotation needed to face the goal
  // Can be adjusted to be in certain scoring spots rather than
  public Transform2d getToScoringPosition() {
    // if in scoring range assuming that the range has a radius
    if (distanceFromGoal(getRobotPose().get().estimatedPose) <= Constants.scoringRange) {
      // return Transform2d staying in same place and just rotating to line up with goal
      return new Transform2d(new Translation2d(), Rotation2d.fromRadians(getRobotPose().get().estimatedPose.getRotation().getZ()));
    } else {
      // hopefully returns a Transform2d that tells robot where to go and how much to rotate by
      return new Transform2d(
        new Translation2d(Math.abs(getClosestScoringPoint().getX() - getRobotPose().get().estimatedPose.getX()),
        Math.abs(getClosestScoringPoint().getY() - getRobotPose().get().estimatedPose.getY())),
        Rotation2d.fromRadians(getRobotPose().get().estimatedPose.getRotation().getZ())
      );
    }
  }
}