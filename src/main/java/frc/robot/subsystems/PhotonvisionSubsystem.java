package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonvisionSubsystem {
  private final PhotonCamera camera;
  private final PhotonPoseEstimator poseEstimator;
  private final Transform3d robotToCam;

  public PhotonvisionSubsystem(String cameraName) {
    this.camera = new PhotonCamera(cameraName);
    this.robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0));
    this.poseEstimator =
        new PhotonPoseEstimator(
            AprilTagFields.k2024Crescendo
                .loadAprilTagLayoutField(), // TODO deprecated please update
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            robotToCam);
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
    return null;
    // return poseEstimator.update(getLatestResults());
    // TODO: the getLatestResult() now returns a list whcih is up to 20 PhotonPipelineResults from a
    // FIFO queue, figure out how to update the pose for every missed one and return the most
    // updated pose, maybe pass in a pose
  }

  public List<PhotonPipelineResult> getLatestResults() {
    return camera.getAllUnreadResults();
  }

  public boolean hasTargets() {
    return getLatestResults().size() != 0 ? getLatestResults().get(0).hasTargets() : false;
  }

  public PhotonTrackedTarget getBestTarget() {
    if (hasTargets()) return getLatestResults().get(0).getBestTarget();
    return null;
  }

  public double getYaw() {
    PhotonTrackedTarget target = getBestTarget();
    if (target != null) {
      return target.getYaw();
    }
    return 0.0;
  }

  public double getPitch() {
    PhotonTrackedTarget target = getBestTarget();
    if (target != null) {
      return target.getPitch();
    }
    return 0.0;
  }

  public double getArea() {
    PhotonTrackedTarget target = getBestTarget();
    if (target != null) {
      return target.getArea();
    }
    return 0.0;
  }

  public double getSkew() {
    PhotonTrackedTarget target = getBestTarget();
    if (target != null) {
      return target.getSkew();
    }
    return 0.0;
  }
}
