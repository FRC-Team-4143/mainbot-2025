// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.mw_lib.proxy_server.PieceDetectionPacket.PieceDetection;
import frc.mw_lib.proxy_server.ProxyServer;
import frc.mw_lib.subsystem.Subsystem;
import frc.robot.Constants;
import frc.robot.Constants.CoralDetectorConstants;
import monologue.Annotations.Log;
import monologue.Logged;

public class CoralDetector extends Subsystem {

  // Singleton pattern
  private static CoralDetector coral_detector_instance_ = null;

  private PieceDetection coral_detection_;
  private StructPublisher<Pose3d> pose_pub_;
  private Debouncer validity_debouncer_;

  public static CoralDetector getInstance() {
    if (coral_detector_instance_ == null) {
      coral_detector_instance_ = new CoralDetector();
    }
    return coral_detector_instance_;
  }

  /** Class Members */
  private CoralDetectorPeriodicIo io_;

  private CoralDetector() {
    // Create io object first in subsystem configuration
    io_ = new CoralDetectorPeriodicIo();

    validity_debouncer_ =
        new Debouncer(CoralDetectorConstants.DETECT_DEBOUNCE_TIME, Debouncer.DebounceType.kBoth);
    pose_pub_ =
        NetworkTableInstance.getDefault()
            .getStructTopic("CoralDetector/Coral Pose", Pose3d.struct)
            .publish();

    // coral_detection_ = new PieceDetection();

    // Call reset last in subsystem configuration
    reset();
  }

  /**
   * This function should be logic and code to fully reset your subsystem. This is called during
   * initialization, and should handle I/O configuration and initializing data members.
   */
  @Override
  public void reset() {}

  /**
   * Inside this function, all of the SENSORS should be read into variables stored in the PeriodicIO
   * class defined below. There should be no calls to output to actuators, or any logic within this
   * function.
   */
  @Override
  public void readPeriodicInputs(double timestamp) {
    if (ProxyServer.getLatestPieceDetections().size() > 0) {
      coral_detection_ = ProxyServer.getLatestPieceDetections().get(0);
      io_.can_see_coral_ = true;
      io_.target_x_ = Units.degreesToRadians(coral_detection_.theta_x_);
      io_.target_y_ = Units.degreesToRadians(coral_detection_.theta_y_);
    } else {
      io_.can_see_coral_ = false;
    }
  }

  /**
   * Inside this function, all of the LOGIC should compute updates to output variables in the
   * PeriodicIO class defined below. There should be no calls to read from sensors or write to
   * actuators in this function.
   */
  @Override
  public void updateLogic(double timestamp) {
    if (io_.can_see_coral_) {
      io_.target_distance_ = calculateDist(io_.target_y_);
      io_.coral_pose_ = calculateCoralPose2d();
    }

    updateValidity();
  }

  /**
   * Inside this function actuator OUTPUTS should be updated from data contained in the PeriodicIO
   * class defined below. There should be little to no logic contained within this function, and no
   * sensors should be read.
   */
  @Override
  public void writePeriodicOutputs(double timestamp) {}

  /**
   * Inside this function telemetry should be output to smartdashboard. The data should be collected
   * out of the PeriodicIO class instance defined below. There should be no sensor information read
   * in this function nor any outputs made to actuators within this function. Only publish to
   * smartdashboard here.
   */
  @Override
  public void outputTelemetry(double timestamp) {
    SmartDashboard.putBoolean("CoralDetector/Can See Coral", io_.can_see_coral_);
    SmartDashboard.putBoolean("CoralDetector/Target Valid", io_.target_valid_);
    SmartDashboard.putNumber("CoralDetector/Target Distance", io_.target_distance_);
    SmartDashboard.putNumber("CoralDetector/Target X", Units.radiansToDegrees(io_.target_x_));
    SmartDashboard.putNumber("CoralDetector/Target Y", Units.radiansToDegrees(io_.target_y_));

    if (io_.can_see_coral_) {
      pose_pub_.set(new Pose3d(getCoralPose2d()));
    } else {
      pose_pub_.set(Pose3d.kZero);
    }
  }

  /**
   * @param y the rotation in rads vertically that the coal is from the center of the cam screen
   * @return the distance in meters the coral is from the lens of the cam along the floor
   */
  public double calculateDist(double y) {
    double angleToGoal = CoralDetectorConstants.CAMERA_TRANSFORM.getRotation().getY() + y;
    return (CoralDetectorConstants.CORAL_HEIGHT_METERS
            - CoralDetectorConstants.CAMERA_TRANSFORM.getZ())
        / Math.tan(angleToGoal);
  }

  /**
   * @param dist the distance in meters the coral is from the lens of the cam along the floor
   * @param x the rotation in rads horizontally that the coal is from the center of the cam screen
   * @return the Translation2d from the robot to the coal
   */
  private Transform2d calculateCoralTransform2d(double dist, double x) {
    Transform3d camTransform = CoralDetectorConstants.CAMERA_TRANSFORM;
    Transform2d coralToCam =
        new Transform2d(new Translation2d(dist, Rotation2d.fromRadians(x)), Rotation2d.kZero);
    Transform2d botToCam =
        new Transform2d(
            camTransform.getX(), camTransform.getY(), camTransform.getRotation().toRotation2d());
    Transform2d finalTransform = coralToCam.plus(botToCam);
    return finalTransform;
  }

  /**
   * @return the Pose2d of the coal on the field
   */
  private Pose2d calculateCoralPose2d() {
    Pose2d coralPose =
        PoseEstimator.getInstance()
            .getRobotPose()
            .transformBy(calculateCoralTransform2d(io_.target_distance_, io_.target_x_));
    Transform2d pickup_path_ =
        new Transform2d(PoseEstimator.getInstance().getRobotPose(), coralPose);
    Rotation2d approuch_angle_ = pickup_path_.getRotation();
    Pose2d final_ = new Pose2d(coralPose.getX(), coralPose.getY(), approuch_angle_);
    return final_;
  }

  public Pose2d getCoralPose2d() {
    return io_.coral_pose_;
  }

  public boolean isValid() {
    return io_.target_valid_;
  }

  private void updateValidity() {
    io_.target_valid_ =
        validity_debouncer_.calculate(
            io_.can_see_coral_
                && (io_.target_distance_
                    < Constants.CoralDetectorConstants.DETECTION_DISTANCE_LIMIT));
  }

  public Transform2d getCoralTransform2d() {
    return calculateCoralTransform2d(io_.target_distance_, io_.target_x_);
  }

  public class CoralDetectorPeriodicIo implements Logged {
    @Log.File double target_x_ = 0.0;
    @Log.File double target_y_ = 0.0;
    @Log.File boolean target_valid_ = false;
    @Log.File double target_distance_ = 0.0;
    @Log.File boolean can_see_coral_ = false;
    @Log.File Pose2d coral_pose_ = new Pose2d();
  }

  public Logged getLoggingObject() {
    return io_;
  }
}
