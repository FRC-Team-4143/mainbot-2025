package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.mw_lib.subsystem.Subsystem;
import frc.robot.Vision;
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
  private StructPublisher<Pose2d> camPosePublisher;

  int update_counter_ = 2;
  double[] vision_std_devs_ = {1, 1, 1};

  PoseEstimator() {
    io_ = new PoseEstimatorPeriodicIo();
    field_ = new Field2d();

    camPosePublisher =
        NetworkTableInstance.getDefault().getStructTopic("CamPose", Pose2d.struct).publish();
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
    // Correct pose estimate with vision measurements from Photonvision
    var vision = Vision.getInstance();
    var visionEst = vision.getEstimatedGlobalPose();
    visionEst.ifPresent(
        est -> {
          // Change our trust in the measurement based on the tags we can see
          var estStdDevs = vision.getEstimationStdDevs();

          var est_timestamp = est.timestampSeconds;
          var now = Timer.getFPGATimestamp();
          var latency = now - est_timestamp;
          SmartDashboard.putNumber("vision latency", latency);
          if (latency < 0 || latency > .05) {
            // keep latency sane
            latency = .05;
            est_timestamp = now - latency;
          }

          vision_filtered_odometry_.addVisionMeasurement(
              est.estimatedPose.toPose2d(), est_timestamp, estStdDevs);

          camPosePublisher.set(est.estimatedPose.toPose2d());
        });
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
  }

  @Override
  public void writePeriodicOutputs(double timestamp) {}

  @Override
  public void outputTelemetry(double timestamp) {
    field_.setRobotPose(io_.vision_filtered_pose_);
    SmartDashboard.putData("Field", field_);
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
    vision_filtered_odometry_.resetPosition(pose.getRotation(), drive.getModulePositions(), pose);
  }

  /** Simulates an external force applied to the robot */
  public void disturbPose() {
    var disturbance =
        new Transform2d(new Translation2d(1.0, 1.0), new Rotation2d(0.17 * 2 * Math.PI));
    setRobotOdometry(getFieldPose().plus(disturbance));
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
