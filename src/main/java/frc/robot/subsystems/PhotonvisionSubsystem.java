package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.constants.Constants;

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
  private Matrix<N3, N1> curStdDevs; 

  public PhotonvisionSubsystem(String cameraName) {
    this.camera = new PhotonCamera(cameraName);
    this.robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0));
    this.poseEstimator =
        new PhotonPoseEstimator(
            AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo),
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            robotToCam);
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
    Optional<EstimatedRobotPose> latestPose = Optional.empty();
    for (PhotonPipelineResult result : getLatestResults()) {
      latestPose = poseEstimator.update(result);
      updateEstimationStdDevs(latestPose, result.getTargets());
    }

    return latestPose;
  }
  private void updateEstimationStdDevs(
        Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
        if (estimatedPose.isEmpty()) {
            // if no pose input then default to the regular single tag devieations
            curStdDevs = Constants.kSingleTagStdDevs;
        } else {
            // start running heurisitc(basically its good enough for pose caluclation)
            var estStdDevs = Constants.kSingleTagStdDevs;
            int numTags = 0;
            double avgDist = 0;
            //precalculation
            for (var tgt : targets) {
                var tagPose = poseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty()) continue;
                numTags++;
                avgDist +=
                        tagPose
                                .get()
                                .toPose2d()
                                .getTranslation()
                                .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
            }

            if (numTags == 0) {
                curStdDevs = Constants.kSingleTagStdDevs;
            } else {
                avgDist /= numTags;
                if (numTags > 1) estStdDevs = Constants.kMultiTagStdDevs;
                if (numTags == 1 && avgDist > 4)
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                curStdDevs = estStdDevs;
            }
        }
    }

  public Matrix<N3, N1> getEstimationStdDevs() { //we can use this later on to add both global pose and std dev pose to drive train pose
      return curStdDevs;
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
