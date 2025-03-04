package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.FieldRegions;
import frc.mw_lib.geometry.PolygonRegion;
import frc.mw_lib.geometry.Region;
import frc.mw_lib.subsystem.Subsystem;
import frc.robot.Vision;
import java.util.ArrayList;
import java.util.Optional;
import monologue.Annotations.Log;
import monologue.Logged;
import org.photonvision.EstimatedRobotPose;

public class PoseEstimator extends Subsystem {

  private static PoseEstimator poseEstimatorInstance;

  public static PoseEstimator getInstance() {
    if (poseEstimatorInstance == null) {
      poseEstimatorInstance = new PoseEstimator();
    }
    return poseEstimatorInstance;
  }

  private PoseEstimatorPeriodicIo io_;
  private Field2d field_;
  private SwerveDrivePoseEstimator vision_filtered_odometry_;
  private StructPublisher<Pose2d> cam_pose_pub_;
  private StructPublisher<Pose2d> robot_pose_pub_;
  private StructArrayPublisher<Transform3d> used_tags_pub_;

  int update_counter_ = 2;
  double[] vision_std_devs_ = {1, 1, 1};

  PoseEstimator() {
    io_ = new PoseEstimatorPeriodicIo();
    field_ = new Field2d();

    cam_pose_pub_ =
        NetworkTableInstance.getDefault()
            .getStructTopic("PoseEstimation/Vision Pose", Pose2d.struct)
            .publish();
    robot_pose_pub_ =
        NetworkTableInstance.getDefault()
            .getStructTopic("PoseEstimation/Robot Pose", Pose2d.struct)
            .publish();
    used_tags_pub_ =
        NetworkTableInstance.getDefault()
            .getStructArrayTopic("PoseEstimation/Tags Detected", Transform3d.struct)
            .publish();
  }

  @Override
  public void reset() {
    vision_filtered_odometry_ =
        new SwerveDrivePoseEstimator(
            SwerveDrivetrain.getInstance().kinematics_,
            new Rotation2d(),
            SwerveDrivetrain.getInstance().getModulePositions(),
            new Pose2d());
  }

  @Override
  public void readPeriodicInputs(double timestamp) {
    if (io_.vision_data_packet_.size() < Vision.getInstance().getNumCameras()) {
      io_.vision_data_packet_.ensureCapacity(Vision.getInstance().getNumCameras());
      for (int i = 0; i < Vision.getInstance().getNumCameras(); i++) {
        io_.vision_data_packet_.add(null);
      }
    }

    for (int i = 0; i < Vision.getInstance().getNumCameras(); i++) {
      io_.vision_data_packet_.set(i, Vision.getInstance().getEstimatedGlobalPose(i));
    }
  }

  // Make a subscriber, integate vision measurements wpilib method on the new
  // odometry, getLastChange?
  @Override
  public void updateLogic(double timestamp) {
    for (Optional<EstimatedRobotPose> elem : io_.vision_data_packet_) {
      elem.ifPresentOrElse(
          est -> {
            // Change our trust in the measurement based on the tags we can see
            var estStdDevs = Vision.getInstance().getEstimationStdDevs();

            var est_timestamp = est.timestampSeconds;
            var latency = timestamp - est_timestamp;
            SmartDashboard.putNumber("Subsystems/PoseEstimator/Vision Latency", latency);
            if (latency < 0 || latency > .05) {
              // keep latency sane
              latency = .05;
              est_timestamp = timestamp - latency;
            }
            io_.raw_vision_pose_ = Optional.of(est.estimatedPose.toPose2d());

            vision_filtered_odometry_.addVisionMeasurement(
                io_.raw_vision_pose_.get(), est_timestamp, estStdDevs);
          },
          () -> io_.raw_vision_pose_ = Optional.empty());

      io_.filtered_vision_pose_ =
          vision_filtered_odometry_.updateWithTime(
              timestamp,
              SwerveDrivetrain.getInstance().getImuYaw(),
              SwerveDrivetrain.getInstance().getModulePositions());
    }
  }

  @Override
  public void writePeriodicOutputs(double timestamp) {}

  @Override
  public void outputTelemetry(double timestamp) {
    field_.setRobotPose(io_.filtered_vision_pose_);
    if (io_.raw_vision_pose_.isPresent()) cam_pose_pub_.set(io_.raw_vision_pose_.get());
    robot_pose_pub_.set(io_.filtered_vision_pose_);
    Transform3d[] tags = new Transform3d[io_.detected_tags_.size()];
    tags = io_.detected_tags_.toArray(tags);
    used_tags_pub_.set(tags);
    SmartDashboard.putData("Subsystems/PoseEstimator/Field", field_);
  }

  public Field2d getFieldWidget() {
    return field_;
  }

  public Pose2d getRobotPose() {
    return io_.filtered_vision_pose_;
  }

  public SwerveDrivePoseEstimator getOdometryPose() {
    return vision_filtered_odometry_;
  }

  /**
   * Resets the robot odom and set the robot pose to supplier Pose
   *
   * @param pose pose to update Robot pose to
   */
  public void setRobotOdometry(Pose2d pose) {
    var drive = SwerveDrivetrain.getInstance();
    SwerveDrivetrain.getInstance().seedFieldRelative(pose.getRotation());
    vision_filtered_odometry_.resetPosition(pose.getRotation(), drive.getModulePositions(), pose);
  }

  /** Simulates an external force applied to the robot */
  public void disturbPose() {
    var disturbance =
        new Transform2d(new Translation2d(1.0, 1.0), new Rotation2d(0.17 * 2 * Math.PI));
    setRobotOdometry(getRobotPose().plus(disturbance));
  }

  /**
   * Returns the robots current load station region or an empty optional if not in any.
   *
   * @return current region
   */
  public Optional<Region> loadStationRegion() {
    for (PolygonRegion region : FieldRegions.STATION_REGIONS) {
      if (region.contains(io_.filtered_vision_pose_)) {
        return Optional.of(region);
      }
    }
    return Optional.empty();
  }

  /**
   * Returns the robots current algae region or an empty optional if not in any.
   *
   * @return current region
   */
  public Optional<Region> algaeRegion() {
    for (PolygonRegion region : FieldRegions.ALGAE_REGIONS) {
      if (region.contains(io_.filtered_vision_pose_)) {
        return Optional.of(region);
      }
    }
    return Optional.empty();
  }

  /**
   * Returns the robots current reef region or an empty optional if not in any.
   *
   * @return current region
   */
  public Optional<Region> reefPose() {
    for (PolygonRegion region : FieldRegions.REEF_REGIONS) {
      if (region.contains(io_.filtered_vision_pose_)) {
        return Optional.of(region);
      }
    }
    return Optional.empty();
  }

  public class PoseEstimatorPeriodicIo implements Logged {
    @Log.File
    public Pose2d filtered_vision_pose_ =
        new Pose2d(
            Units.inchesToMeters(100.003),
            Units.inchesToMeters(158.500),
            Rotation2d.fromDegrees(0)); // new Pose2d();

    @Log.File public Optional<Pose2d> raw_vision_pose_ = Optional.empty();

    @Log.File
    public ArrayList<Optional<EstimatedRobotPose>> vision_data_packet_ =
        new ArrayList<Optional<EstimatedRobotPose>>();
  }

  @Override
  public Logged getLoggingObject() {
    return io_;
  }
}
