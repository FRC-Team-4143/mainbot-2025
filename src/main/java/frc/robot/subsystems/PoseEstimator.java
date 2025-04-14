package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.FieldRegions;
import frc.mw_lib.geometry.PolygonRegion;
import frc.mw_lib.geometry.Region;
import frc.mw_lib.subsystem.Subsystem;
import frc.robot.Constants;
import frc.robot.PhotonVision;
import java.util.ArrayList;
import java.util.List;
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
  private StructArrayPublisher<Pose3d> used_tags_pub_;

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
            .getStructArrayTopic("PoseEstimation/Tags Detected", Pose3d.struct)
            .publish();
  }

  @Override
  public void reset() {
    vision_filtered_odometry_ =
        new SwerveDrivePoseEstimator(
            SwerveDrivetrain.getInstance().kinematics_,
            new Rotation2d(),
            SwerveDrivetrain.getInstance().getModulePositions(),
            new Pose2d(2, 4, Rotation2d.fromDegrees(180)));
  }

  @Override
  public void readPeriodicInputs(double timestamp) {
    int num_packets = io_.vision_data_packet_.size();
    if (num_packets < PhotonVision.getInstance().getNumCameras()) {
      io_.vision_data_packet_.ensureCapacity(PhotonVision.getInstance().getNumCameras());
      for (int i = num_packets; i < PhotonVision.getInstance().getNumCameras(); i++) {
        io_.vision_data_packet_.add(null);
      }
    }

    for (int i = 0; i < PhotonVision.getInstance().getNumCameras(); i++) {
      io_.vision_data_packet_.set(i, PhotonVision.getInstance().getEstimatedGlobalPose(i));
    }
  }

  // Make a subscriber, integate vision measurements wpilib method on the new
  // odometry, getLastChange?
  @Override
  public void updateLogic(double timestamp) {
    for (int i = 0; i < io_.vision_data_packet_.size(); i++) {
      if (io_.vision_data_packet_.get(i).isPresent()) {
        var est = io_.vision_data_packet_.get(i).get();

        // Change our trust in the measurement based on the tags we can see
        var estStdDevs = PhotonVision.getInstance().getEstimationStdDevs();

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

        if (io_.detected_tags_.size() <= i) {
          for (int j = io_.detected_tags_.size(); j <= i; j++) {
            io_.detected_tags_.add(new ArrayList<>());
          }
        }
        io_.detected_tags_.get(i).clear();
        for (var target : est.targetsUsed) {
          Optional<Pose3d> tagPose = Constants.Vision.TAG_LAYOUT.getTagPose(target.fiducialId);
          if (tagPose.isPresent()) {
            io_.detected_tags_.get(i).add(tagPose.get());
          }
        }
      } else {
        io_.raw_vision_pose_ = Optional.empty();
      }

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
    io_.raw_vision_pose_.ifPresent((pose) -> cam_pose_pub_.set(pose));
    robot_pose_pub_.set(io_.filtered_vision_pose_);
    SmartDashboard.putData("Subsystems/PoseEstimator/Field", field_);
    // Publish Detected Tags as Vision Targets
    ArrayList<Pose3d> allTags = new ArrayList<>();
    for (List<Pose3d> array : io_.detected_tags_) allTags.addAll(array);

    Pose3d[] tags = new Pose3d[allTags.size()];
    tags = allTags.toArray(tags);
    used_tags_pub_.set(tags);
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
      if (region.contains(getRobotPose())) {
        return Optional.of(region);
      }
    }
    return Optional.empty();
  }

  public boolean isBargeZone() {
    return FieldRegions.BARGE_REGION.contains(getRobotPose());
  }

  public boolean isProcessorZone() {
    return FieldRegions.PROCESSOR_REGION.contains(getRobotPose());
  }

  public boolean isStationZone() {
    return FieldRegions.LEFT_CORAL_STATION_REGION.contains(getRobotPose())
        || FieldRegions.RIGHT_CORAL_STATION_REGION.contains(getRobotPose());
  }

  public boolean isInL4CollisionZone() {
    return FieldRegions.L4_COLLISION_REGION.contains(getRobotPose());
  }

  /**
   * Returns the robots current reef region or an empty optional if not in any.
   *
   * @return current region
   */
  public Optional<Region> reefPose() {
    for (PolygonRegion region : FieldRegions.REEF_REGIONS) {
      if (region.contains(getRobotPose())) {
        return Optional.of(region);
      }
    }
    return Optional.empty();
  }

  public class PoseEstimatorPeriodicIo implements Logged {
    @Log.File public Pose2d filtered_vision_pose_ = new Pose2d();
    @Log.File public Optional<Pose2d> raw_vision_pose_ = Optional.empty();

    @Log.File
    public ArrayList<Optional<EstimatedRobotPose>> vision_data_packet_ =
        new ArrayList<Optional<EstimatedRobotPose>>();

    @Log.File public List<List<Pose3d>> detected_tags_ = new ArrayList<>();
  }

  @Override
  public Logged getLoggingObject() {
    return io_;
  }
}
