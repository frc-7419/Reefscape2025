/*
 * MIT License
 *
 * Copyright (c) PhotonVision
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package frc.robot.subsystems;

import static frc.robot.constants.Constants.VisionConstants.kMultiTagStdDevs;
import static frc.robot.constants.Constants.VisionConstants.kSingleTagStdDevs;
import static frc.robot.constants.Constants.VisionConstants.kTagLayout;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Robot;
import frc.robot.constants.Constants.CameraConfig;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionSubsystem {
  public static class VisionResult {
    public final EstimatedRobotPose estimatedRobotPose;
    public final Matrix<N3, N1> stdDevs;
    public final String cameraName;

    public VisionResult(
        EstimatedRobotPose estimatedRobotPose, Matrix<N3, N1> stdDevs, String cameraName) {
      this.estimatedRobotPose = estimatedRobotPose;
      this.stdDevs = stdDevs;
      this.cameraName = cameraName;
    }
  }

  private final List<PhotonCamera> cameras = new ArrayList<>();
  private final List<PhotonPoseEstimator> photonEstimators = new ArrayList<>();

  private Matrix<N3, N1> curStdDevs;

  // Simulation
  private final List<PhotonCameraSim> cameraSims = new ArrayList<>();
  private VisionSystemSim visionSim;

  public VisionSubsystem(List<CameraConfig> cameraConfigs) {
    if (Robot.isSimulation()) {
      visionSim = new VisionSystemSim("main");
      visionSim.addAprilTags(kTagLayout);
    }

    for (CameraConfig config : cameraConfigs) {
      PhotonCamera camera = new PhotonCamera(config.name);
      cameras.add(camera);

      PhotonPoseEstimator estimator = new PhotonPoseEstimator(
          kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, config.cameraToRobot);
      estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
      photonEstimators.add(estimator);

      if (Robot.isSimulation()) {
        SimCameraProperties cameraProp = new SimCameraProperties();
        cameraProp.setCalibration(1280, 960, Rotation2d.fromDegrees(90));
        cameraProp.setCalibError(0.25, 0.10);
        cameraProp.setFPS(60);
        cameraProp.setAvgLatencyMs(10);
        cameraProp.setLatencyStdDevMs(10);

        PhotonCameraSim cameraSim = new PhotonCameraSim(camera, cameraProp);
        visionSim.addCamera(cameraSim, config.cameraToRobot);
        cameraSim.enableDrawWireframe(true);

        cameraSims.add(cameraSim);
      }
    }
  }

  /**
   * The latest estimated robot pose on the field from vision data. This may be
   * empty. This should
   * only be called once per loop.
   *
   * <p>
   * Also includes updates for the standard deviations, which can (optionally) be
   * retrieved with
   * {@link getEstimationStdDevs}
   *
   * @return An {@link EstimatedRobotPose} with an estimated pose, estimate
   *         timestamp, and targets
   *         used for estimation.
   */
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
    List<PhotonTrackedTarget> allCameraTargets = new ArrayList<>();
    List<EstimatedRobotPose> allVisionEstimates = new ArrayList<>();

    for (int i = 0; i < cameras.size(); i++) {
      final int index = i;
      PhotonCamera camera = cameras.get(i);
      PhotonPoseEstimator estimator = photonEstimators.get(i);

      var allResults = camera.getAllUnreadResults();
      for (var pipelineResult : allResults) {
        allCameraTargets.addAll(pipelineResult.getTargets());

        Optional<EstimatedRobotPose> visionEst = estimator.update(pipelineResult);

        if (visionEst.isPresent()) {
          allVisionEstimates.add(visionEst.get());
        }

        if (Robot.isSimulation()) {
          visionEst.ifPresentOrElse(
              est -> getSimDebugField()
                  .getObject("VisionEstimation" + index)
                  .setPose(est.estimatedPose.toPose2d()),
              () -> getSimDebugField().getObject("VisionEstimation" + index).setPoses());
        }
      }
    }

    updateEstimationStdDevs(
        allVisionEstimates.isEmpty()
            ? Optional.empty()
            : Optional.of(allVisionEstimates.get(allVisionEstimates.size() - 1)),
        allCameraTargets,
        photonEstimators.get(0));

    return averageVisionEstimates(allVisionEstimates);
  }

  private Optional<EstimatedRobotPose> averageVisionEstimates(List<EstimatedRobotPose> estimates) {
    if (estimates.isEmpty()) {
      return Optional.empty();
    }

    double x = 0.0, y = 0.0, rotation = 0.0;
    double latestTimestamp = 0.0;
    List<PhotonTrackedTarget> combinedTargets = new ArrayList<>();
    Set<Integer> seenTargets = new HashSet<>();

    for (EstimatedRobotPose estimate : estimates) {
      Pose2d pose = estimate.estimatedPose.toPose2d();
      x += pose.getX();
      y += pose.getY();
      rotation += pose.getRotation().getRadians();

      if (estimate.timestampSeconds > latestTimestamp) {
        latestTimestamp = estimate.timestampSeconds;
      }

      for (PhotonTrackedTarget target : estimate.targetsUsed) {
        if (seenTargets.add(target.getFiducialId())) {
          combinedTargets.add(target);
        }
      }
    }

    x /= estimates.size();
    y /= estimates.size();
    rotation /= estimates.size();

    Pose3d avgPose = new Pose3d(x, y, 0, new Rotation3d(0, 0, rotation));

    return Optional.of(
        new EstimatedRobotPose(
            avgPose, latestTimestamp, combinedTargets, estimates.get(0).strategy));
  }

  /**
   * Calculates new standard deviations This algorithm is a heuristic that creates
   * dynamic standard
   * deviations based on number of tags, estimation strategy, and distance from
   * the tags.
   *
   * @param estimatedPose The estimated pose to guess standard deviations for.
   * @param targets       All targets in this camera frame
   */
  private void updateEstimationStdDevs(
      Optional<EstimatedRobotPose> estimatedPose,
      List<PhotonTrackedTarget> targets,
      PhotonPoseEstimator estimator) {

    if (estimatedPose.isEmpty()) {
      curStdDevs = kSingleTagStdDevs;
    } else {
      var estStdDevs = kSingleTagStdDevs;
      int numTags = 0;
      double avgDist = 0;

      for (var tgt : targets) {
        var tagPose = estimator.getFieldTags().getTagPose(tgt.getFiducialId());
        if (tagPose.isEmpty())
          continue;
        numTags++;
        avgDist += tagPose
            .get()
            .toPose2d()
            .getTranslation()
            .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
      }

      if (numTags == 0) {
        curStdDevs = kSingleTagStdDevs;
      } else {
        avgDist /= numTags;

        if (numTags > 1) {
          estStdDevs = kMultiTagStdDevs;
        }

        if (numTags == 1 && avgDist > 4) {
          estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        } else {
          estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
        }
        curStdDevs = estStdDevs;
      }
    }
  }

  /**
   * Returns the latest standard deviations of the estimated pose from {@link
   * #getEstimatedGlobalPose()}, for use with {@link
   * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
   * SwerveDrivePoseEstimator}. This should
   * only be used when there are targets visible.
   */
  public Matrix<N3, N1> getEstimationStdDevs() {
    return curStdDevs;
  }

  // ----- Simulation

  public void simulationPeriodic(Pose2d robotSimPose) {
    visionSim.update(robotSimPose);
  }

  /** Reset pose history of the robot in the vision system simulation. */
  public void resetSimPose(Pose2d pose) {
    if (Robot.isSimulation())
      visionSim.resetRobotPose(pose);
  }

  /** A Field2d for visualizing our robot and objects on the field. */
  public Field2d getSimDebugField() {
    if (!Robot.isSimulation())
      return null;
    return visionSim.getDebugField();
  }

  private Matrix<N3, N1> computeStdDevs(
      Optional<EstimatedRobotPose> estimatedPoseOpt,
      List<PhotonTrackedTarget> targets,
      PhotonPoseEstimator estimator) {
    if (estimatedPoseOpt.isEmpty()) {
      return kSingleTagStdDevs;
    } else {
      var estStdDevs = kSingleTagStdDevs;
      int numTags = 0;
      double avgDist = 0;

      EstimatedRobotPose estimatedPose = estimatedPoseOpt.get();
      for (var tgt : targets) {
        var tagPoseOpt = estimator.getFieldTags().getTagPose(tgt.getFiducialId());
        if (tagPoseOpt.isEmpty())
          continue;
        numTags++;
        avgDist += tagPoseOpt
            .get()
            .toPose2d()
            .getTranslation()
            .getDistance(estimatedPose.estimatedPose.toPose2d().getTranslation());
      }
      if (numTags == 0) {
        return kSingleTagStdDevs;
      } else {
        avgDist /= numTags;
        if (numTags > 1) {
          estStdDevs = kMultiTagStdDevs;
        }
        if (numTags == 1 && avgDist > 4) {
          estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        } else {
          estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
        }
        return estStdDevs;
      }
    }
  }

  public List<VisionResult> getIndividualVisionEstimates() {
    List<VisionResult> visionResults = new ArrayList<>();
    for (int i = 0; i < cameras.size(); i++) {
      PhotonCamera camera = cameras.get(i);
      PhotonPoseEstimator estimator = photonEstimators.get(i);

      var allResults = camera.getAllUnreadResults();
      for (var pipelineResult : allResults) {
        Optional<EstimatedRobotPose> visionEst = estimator.update(pipelineResult);
        if (visionEst.isPresent()) {
          Matrix<N3, N1> stdDevs = computeStdDevs(visionEst, pipelineResult.getTargets(), estimator);
          visionResults.add(new VisionResult(visionEst.get(), stdDevs, camera.getName()));

          if (Robot.isSimulation()) {
            final int index = i;
            visionEst.ifPresentOrElse(
                est -> getSimDebugField()
                    .getObject("VisionEstimation" + index)
                    .setPose(est.estimatedPose.toPose2d()),
                () -> getSimDebugField().getObject("VisionEstimation" + index).setPoses());
          }
        }
      }
    }
    return visionResults;
  }
}
