package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rotation3d;

import edu.wpi.first.apriltag.AprilTagFields;

public class PhotonvisionSubsystem {
    private final PhotonCamera camera;
    private final PhotonPoseEstimator poseEstimator;
    private final Transform3d robotToCam;

    public PhotonvisionSubsystem(String cameraName) {
        this.camera = new PhotonCamera(cameraName);
        this.robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0));
        this.poseEstimator = new PhotonPoseEstimator(AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(),
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCam);
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        return poseEstimator.update(getLatestResult());
    }

    public PhotonPipelineResult getLatestResult() {
        return camera.getLatestResult();
    }

    public boolean hasTargets() {
        return getLatestResult().hasTargets();
    }

    public PhotonTrackedTarget getBestTarget() {
        if (hasTargets()) {
            return getLatestResult().getBestTarget();
        }
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