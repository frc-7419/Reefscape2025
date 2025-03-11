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
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Robot;
import frc.robot.constants.Constants.CameraConfig;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
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

      PhotonPoseEstimator estimator =
          new PhotonPoseEstimator(
              kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, config.cameraToRobot);
      estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
      photonEstimators.add(estimator);

      if (Robot.isSimulation()) {
        SimCameraProperties cameraProp = new SimCameraProperties();
        cameraProp.setCalibration(800, 600, Rotation2d.fromDegrees(75));
        cameraProp.setCalibError(0.35, 0.10);
        cameraProp.setFPS(60);
        cameraProp.setAvgLatencyMs(50);
        cameraProp.setLatencyStdDevMs(10);

        PhotonCameraSim cameraSim = new PhotonCameraSim(camera, cameraProp);
        visionSim.addCamera(cameraSim, config.cameraToRobot);
        cameraSim.enableDrawWireframe(true);

        cameraSims.add(cameraSim);
      }
    }
  }

  private Matrix<N3, N1> computeStdDevs(
      Optional<EstimatedRobotPose> estimatedPoseOpt,
      List<PhotonTrackedTarget> targets,
      PhotonPoseEstimator estimator) {
    if (estimatedPoseOpt.isEmpty()) {
      return kSingleTagStdDevs;
    }

    EstimatedRobotPose estimatedPose = estimatedPoseOpt.get();
    Matrix<N3, N1> estStdDevs = kSingleTagStdDevs;
    int numTags = 0;
    double avgDist = 0;

    for (PhotonTrackedTarget tgt : targets) {
      Optional<Pose3d> tagPoseOpt = estimator.getFieldTags().getTagPose(tgt.getFiducialId());
      if (tagPoseOpt.isEmpty()) continue;
      numTags++;
      avgDist +=
          tagPoseOpt
              .get()
              .toPose2d()
              .getTranslation()
              .getDistance(estimatedPose.estimatedPose.toPose2d().getTranslation());
    }

    if (numTags == 0) {
      return kSingleTagStdDevs;
    }

    avgDist /= numTags;

    if (numTags > 1) {
      estStdDevs = kMultiTagStdDevs;
    }

    if (numTags == 1 && avgDist > 4) {
      return VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    } else {
      return estStdDevs.times(1 + (avgDist * avgDist / 30));
    }
  }

  public List<VisionResult> getIndividualVisionEstimates() {
    List<VisionResult> visionResults = new ArrayList<>();
    for (int i = 0; i < cameras.size(); i++) {
      PhotonCamera camera = cameras.get(i);
      PhotonPoseEstimator estimator = photonEstimators.get(i);

      var allResults = camera.getAllUnreadResults();
      for (var pipelineResult : allResults) {
        Optional<EstimatedRobotPose> visionEstOpt = estimator.update(pipelineResult);
        if (visionEstOpt.isPresent()) {
          Matrix<N3, N1> stdDevs =
              computeStdDevs(visionEstOpt, pipelineResult.getTargets(), estimator);
          visionResults.add(new VisionResult(visionEstOpt.get(), stdDevs, camera.getName()));

          if (Robot.isSimulation()) {
            final int index = i;
            visionEstOpt.ifPresentOrElse(
                est ->
                    getSimDebugField()
                        .getObject("VisionEstimation" + index)
                        .setPose(est.estimatedPose.toPose2d()),
                () -> getSimDebugField().getObject("VisionEstimation" + index).setPoses());
          }
        }
      }
    }
    return visionResults;
  }

  public void addHeadingData(double timestamp, Rotation2d heading) {
    for (PhotonPoseEstimator estimator : photonEstimators) {
      estimator.addHeadingData(timestamp, heading);
    }
  }

  public void simulationPeriodic(Pose2d robotSimPose) {
    visionSim.update(robotSimPose);
  }

  /** Reset pose history of the robot in the vision system simulation. */
  public void resetSimPose(Pose2d pose) {
    if (Robot.isSimulation()) {
      visionSim.resetRobotPose(pose);
    }
  }

  /** A Field2d for visualizing our robot and objects on the field. */
  public Field2d getSimDebugField() {
    if (!Robot.isSimulation()) {
      return null;
    }
    return visionSim.getDebugField();
  }
}
