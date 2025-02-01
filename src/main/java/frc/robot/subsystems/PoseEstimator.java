package frc.robot.subsystems;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.ProtobufPublisher;
import edu.wpi.first.networktables.ProtobufSubscriber;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.TimestampedObject;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.FieldConstants;
import frc.mw_lib.geometry.PolygonRegion;
import frc.mw_lib.subsystem.Subsystem;
import monologue.Annotations.Log;
import monologue.Logged;

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
  private SwerveDrivePoseEstimator odometry_;
  private SwerveDrivePoseEstimator vision_filtered_odometry_;
  private ProtobufSubscriber<Pose2d> vision_subsciber_;
  private BooleanPublisher supression_publisher_;
  private BooleanSubscriber vision_ready_subscriber_;
  private ProtobufPublisher<Pose2d> odom_publisher_;
  private ProtobufPublisher<Pose2d> pose_publisher_;
  private DoubleArraySubscriber vision_std_devs_subscriber;
  private PolygonRegion currentAlgaeRegion;
  private PolygonRegion currentCoralRegion;
  private StructPublisher<Pose2d> camPosePublisher;

  int update_counter_ = 2;
  double[] vision_std_devs_ = {1, 1, 1};

  PoseEstimator() {
    io_ = new PoseEstimatorPeriodicIo();
    field_ = new Field2d();

    NetworkTableInstance nti = NetworkTableInstance.getDefault();
    var field_pose_topic = nti.getProtobufTopic("vision/pose", Pose2d.proto);
    var robot_odom_topic = nti.getProtobufTopic("vision/odom", Pose2d.proto);
    var robot_pose_topic = nti.getProtobufTopic("robotpose", Pose2d.proto);
    var supress_odom_topic = nti.getBooleanTopic("vision/supress_odom");
    var vision_ready_topic = nti.getBooleanTopic("vision/ready");
    var vision_std_devs_topic = nti.getDoubleArrayTopic("vision/std_devs");

    vision_subsciber_ = field_pose_topic.subscribe(new Pose2d());
    vision_ready_subscriber_ = vision_ready_topic.subscribe(false);
    vision_std_devs_subscriber = vision_std_devs_topic.subscribe(null);
    odom_publisher_ = robot_odom_topic.publish();
    pose_publisher_ = robot_pose_topic.publish();
    supression_publisher_ = supress_odom_topic.publish();
  }

  @Override
  public void reset() {
    odometry_ =
        new SwerveDrivePoseEstimator(
            SwerveDrivetrain.getInstance().kinematics,
            new Rotation2d(),
            SwerveDrivetrain.getInstance().getModulePositions(),
            new Pose2d());
    vision_filtered_odometry_ =
        new SwerveDrivePoseEstimator(
            SwerveDrivetrain.getInstance().kinematics,
            new Rotation2d(),
            SwerveDrivetrain.getInstance().getModulePositions(),
            new Pose2d());
  }

  @Override
  public void readPeriodicInputs(double timestamp) {
    TimestampedObject<Pose2d> result = vision_subsciber_.getAtomic();
    io_.vision_ready_status_ = vision_ready_subscriber_.get(false);
    vision_std_devs_ = vision_std_devs_subscriber.get(new double[] {1, 1, 1});

    if (result.timestamp > io_.last_vision_timestamp_
        && io_.vision_ready_status_
        && !io_.ignore_vision) {
      vision_filtered_odometry_.addVisionMeasurement(
          result.value.transformBy(new Transform2d(0, 0, new Rotation2d())),
          timestamp - 0.04,
          MatBuilder.fill(
              Nat.N3(), Nat.N1(), vision_std_devs_[0], vision_std_devs_[1], vision_std_devs_[2]));
      io_.last_vision_timestamp_ = result.timestamp;
    }
  }

  // Make a subscriber, integate vision measurements wpilib method on the new
  // odometry, getLastChange?
  @Override
  public void updateLogic(double timestamp) {
    var drive = SwerveDrivetrain.getInstance();
    io_.odom_pose_ = odometry_.update(drive.getImuYaw(), drive.getModulePositions());
    io_.vision_filtered_pose_ =
        vision_filtered_odometry_.updateWithTime(
            timestamp, drive.getImuYaw(), drive.getModulePositions());

    // finds the algae
    for (PolygonRegion region : FieldConstants.ALGAE_REGIONS)
      if (region.contains(getFieldPose())) {
        currentAlgaeRegion = region;
      }

    // finds the coral region
    for (PolygonRegion region : FieldConstants.CORAL_REGIONS)
      if (region.contains(getFieldPose())) {
        currentCoralRegion = region;
      }
  }

  @Override
  public void writePeriodicOutputs(double timestamp) {

    if (DriverStation.isDisabled() && io_.vision_paused) {
      supression_publisher_.set(true);
    } else {
      supression_publisher_.set(!io_.vision_ready_status_);
    }

    if (DriverStation.isDisabled()) {
      update_counter_--;
      if (update_counter_ <= 0) {
        odom_publisher_.set(io_.odom_pose_);
        update_counter_ = 50;
      }
    } else {
      odom_publisher_.set(io_.odom_pose_);
    }
  }

  @Override
  public void outputTelemetry(double timestamp) {
    field_.setRobotPose(io_.vision_filtered_pose_);
    // pose_publisher_.set(io_.vision_filtered_pose_);

    SmartDashboard.putData("Field", field_);
    // SmartDashboard.putBoolean("Is Vision Paused", io_.ignore_vision);

    // outputs the current reigon to smart dashboard
    SmartDashboard.putString("Coral Region", currentCoralRegion.getName());
    SmartDashboard.putString("Coral Region", currentAlgaeRegion.getName());
  }

  public Field2d getFieldWidget() {
    return field_;
  }

  public Pose2d getFieldPose() {
    return io_.vision_filtered_pose_;
  }

  public Pose2d getOdomPose() {
    return io_.odom_pose_;
  }

  // public boolean isRobotInMidFeild() {
  // return Util.epislonEquals(io_.vision_filtered_pose_.getX(), 8.29564, 2.423);
  // // 8.29564 is
  // the center field line
  // // | 2.423 is
  // the center line to
  // // wing line
  // }

  // public boolean isCloseToTarget() {
  // return Util.epislonEquals(io_.vision_filtered_pose_.getTranslation(),
  // ShooterSubsystem.getInstance().getTarget().toPose2d().getTranslation(), 1.9);
  // }

  public SwerveDrivePoseEstimator getOdometryPose() {
    return vision_filtered_odometry_;
  }

  public void pauseVisionFilter() {
    io_.vision_paused = !io_.vision_paused;
  }

  public void setIgnoreVision(boolean ignore) {
    io_.ignore_vision = ignore;
  }

  /**
   * Resets the robot odom and set the robot pose to supplier Pose
   *
   * @param pose pose to update Robot pose to
   */
  public void setRobotOdometry(Pose2d pose) {
    var drive = SwerveDrivetrain.getInstance();
    SwerveDrivetrain.getInstance().seedFieldRelative(pose.getRotation());
    // vision_filtered_odometry_.resetPosition(drive.getImuYaw(),
    // drive.getModulePositions(), pose);
    vision_filtered_odometry_.resetPosition(pose.getRotation(), drive.getModulePositions(), pose);
  }

  public class PoseEstimatorPeriodicIo implements Logged {
    @Log.File public Pose2d odom_pose_ = new Pose2d();
    @Log.File public Pose2d vision_filtered_pose_ = new Pose2d();
    @Log.File public double last_vision_timestamp_ = 0.0;
    @Log.File public boolean vision_ready_status_ = false;
    @Log.File public boolean vision_paused = false;
    @Log.File public boolean ignore_vision = false;
  }

  @Override
  public Logged getLoggingObject() {
    return io_;
  }
}
