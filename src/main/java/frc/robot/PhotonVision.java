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

package frc.robot;

import static frc.robot.Constants.Vision.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.mw_lib.util.CamConstants;
import frc.robot.subsystems.PoseEstimator;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonVision {
  // Singleton pattern
  private static PhotonVision visionInstance = null;

  public static PhotonVision getInstance() {
    if (visionInstance == null) {
      visionInstance = new PhotonVision();
    }
    return visionInstance;
  }

  private final PhotonCamera[] cameras;
  private final PhotonPoseEstimator[] photonEstimators;
  private Matrix<N3, N1> curStdDevs;
  private int numTags;

  public PhotonVision() {
    int cameras_size = Constants.Vision.CAMERAS.size();

    cameras = new PhotonCamera[cameras_size];
    photonEstimators = new PhotonPoseEstimator[cameras_size];

    for (int i = 0; i < cameras_size; i++) {
      CamConstants config = Constants.Vision.CAMERAS.get(i);
      cameras[i] = new PhotonCamera(config.camera_name);
      // DataLogManager.log("Registering camera " + config.camera_name);
      photonEstimators[i] =
          new PhotonPoseEstimator(
              TAG_LAYOUT, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, config.camera_transform);
      photonEstimators[i].setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }
  }

  /**
   * The latest estimated robot pose on the field from vision data. This may be empty. This should
   * only be called once per loop.
   *
   * <p>Also includes updates for the standard deviations, which can (optionally) be retrieved with
   * {@link getEstimationStdDevs}
   *
   * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets
   *     used for estimation.
   */
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(int index) {
    Optional<EstimatedRobotPose> visionEst = Optional.empty();
    for (PhotonPipelineResult change : cameras[index].getAllUnreadResults()) {
      SmartDashboard.putBoolean("Vision/MultiTag" + index, change.getMultiTagResult().isPresent());
      SmartDashboard.putBoolean(
          "Vision/Multitag Failure" + index,
          change.getMultiTagResult().isEmpty() && change.targets.size() > 1);
      SmartDashboard.putBoolean(
          "Vision/Single Tag" + index,
          change.getMultiTagResult().isEmpty() && change.targets.size() == 1);

      visionEst = photonEstimators[index].update(change);
      updateEstimationStdDevs(visionEst, change.getTargets(), index);
    }
    visionEst.ifPresentOrElse(
        (est) -> {
          var rotation = est.estimatedPose.getRotation().toRotation2d().getDegrees();
          var x = est.estimatedPose.getX();
          var y = est.estimatedPose.getY();

          var vision_filtered_odometry = PoseEstimator.getInstance().getRobotPose();
          SmartDashboard.putBoolean(
              "Vision/Rotation Error" + index,
              Math.abs(rotation - vision_filtered_odometry.getRotation().getDegrees()) > 5);
        },
        () -> {
          SmartDashboard.putBoolean("Vision/Rotation Error" + index, false);
        });
    return visionEst;
  }

  public int getNumCameras() {
    return cameras.length;
  }

  /**
   * Calculates new standard deviations This algorithm is a heuristic that creates dynamic standard
   * deviations based on number of tags, estimation strategy, and distance from the tags.
   *
   * @param estimatedPose The estimated pose to guess standard deviations for.
   * @param targets All targets in this camera frame
   */
  private void updateEstimationStdDevs(
      Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets, int index) {
    if (estimatedPose.isEmpty()) {
      // No pose input. Default to single-tag std devs
      curStdDevs = SINGLE_TAG_STD_DEVS;

    } else {
      // Pose present. Start running Heuristic
      var estStdDevs = SINGLE_TAG_STD_DEVS;
      // int numTags = 0;
      numTags = 0;
      double avgDist = 0;

      // Precalculation - see how many tags we found, and calculate an average-distance metric
      for (var tgt : targets) {
        var tagPose = photonEstimators[index].getFieldTags().getTagPose(tgt.getFiducialId());
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
        // No tags visible. Default to single-tag std devs
        curStdDevs = SINGLE_TAG_STD_DEVS;
      } else {
        // One or more tags visible, run the full heuristic.
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1) estStdDevs = MULTI_TAG_STD_DEVS;
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4)
          estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
        curStdDevs = estStdDevs;
      }
    }
  }

  /**
   * Returns the latest standard deviations of the estimated pose from {@link
   * #getEstimatedGlobalPose()}, for use with {@link
   * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}. This should
   * only be used when there are targets visible.
   */
  public Matrix<N3, N1> getEstimationStdDevs() {
    return curStdDevs;
  }
}
