// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.mw_lib.proxy_server.PieceDetectionPacket.PieceDetection;
import frc.mw_lib.proxy_server.ProxyServer;
import frc.mw_lib.subsystem.Subsystem;
import frc.mw_lib.util.Util;
import frc.robot.Constants;
import frc.robot.Constants.CoralDetectorConstants;
import monologue.Annotations.Log;
import monologue.Logged;

public class CoralDetector extends Subsystem {

  // Singleton pattern
  private static CoralDetector coral_detector_instance_ = null;

  private PieceDetection coral_detection_;
  private StructPublisher<Pose3d> pose_pub_;
  private Debouncer validity_debouncer_rising_;
  private Debouncer validity_debouncer_falling_;
  private Transform2d bot_to_cam_tf_;

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

    validity_debouncer_rising_ =
        new Debouncer(
            CoralDetectorConstants.DETECT_DEBOUNCE_TIME_RISING, Debouncer.DebounceType.kRising);
    validity_debouncer_falling_ =
        new Debouncer(
            CoralDetectorConstants.DETECT_DEBOUNCE_TIME_FALLING, Debouncer.DebounceType.kFalling);
    pose_pub_ =
        NetworkTableInstance.getDefault()
            .getStructTopic("CoralDetector/Coral Pose", Pose3d.struct)
            .publish();

    bot_to_cam_tf_ = Util.flatten(CoralDetectorConstants.BOT_TO_CAM_TRANSFORM);

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
      // Validated 4/13 OK (CJT)
      io_.target_x_ = -Units.degreesToRadians(coral_detection_.theta_x_);
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
      io_.cam_to_coral_transform_ = getCamToCoralTf(io_.target_distance_, io_.target_x_);
      io_.bot_to_coral_transform_ = bot_to_cam_tf_.plus(io_.cam_to_coral_transform_);

      // Validated 4/13 (CJT) working as expected
      Pose2d robot_pose = PoseEstimator.getInstance().getRobotPose();
      Pose2d coral_pose =
          robot_pose.transformBy(
              new Transform2d(io_.bot_to_coral_transform_.getTranslation(), Rotation2d.kZero));

      // Validated 4/13 (CJT) coral points to center of robot
      io_.attack_angle_ =
          new Rotation2d(io_.bot_to_coral_transform_.getX(), io_.bot_to_coral_transform_.getY())
              .plus(Rotation2d.kPi);
      io_.coral_pose_ = // coral_pose;
          new Pose2d(coral_pose.getTranslation(), coral_pose.getRotation().plus(io_.attack_angle_));
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
    SmartDashboard.putNumber("CoralDetector/Target tX", Units.radiansToDegrees(io_.target_x_));
    SmartDashboard.putNumber("CoralDetector/Target tY", Units.radiansToDegrees(io_.target_y_));
    SmartDashboard.putNumber("CoralDetector/Camera x", io_.cam_to_coral_transform_.getX());
    SmartDashboard.putNumber("CoralDetector/Camera y", io_.cam_to_coral_transform_.getY());
    SmartDashboard.putNumber("CoralDetector/Target x", io_.bot_to_coral_transform_.getX());
    SmartDashboard.putNumber("CoralDetector/Target y", io_.bot_to_coral_transform_.getY());
    SmartDashboard.putNumber("CoralDetector/Target Attack", io_.attack_angle_.getDegrees());

    if (io_.can_see_coral_) {
      pose_pub_.set(convertToPose3d(io_.coral_pose_));
    } else {
      pose_pub_.set(Pose3d.kZero);
    }
  }

  /**
   * @param y the rotation in rads vertically that the coal is from the center of the cam screen
   * @return the distance in meters the coral is from the lens of the cam along the floor
   */
  public double calculateDist(double y) {
    // Theta = 90deg - (cam_pitch + piece_y_angle)
    // Height = cam_z - coral_height
    // Distance = tan(theta) * Height

    // Validated 4/13 (CJT) mostly works. seems to under / over represent based on
    // perspective as
    // center shifts

    double theta =
        (Math.PI / 2) - (CoralDetectorConstants.BOT_TO_CAM_TRANSFORM.getRotation().getY() + y);
    double height =
        CoralDetectorConstants.BOT_TO_CAM_TRANSFORM.getZ()
            - CoralDetectorConstants.CORAL_HEIGHT_METERS;
    return Math.tan(theta) * height;
  }

  public Pose3d convertToPose3d(Pose2d p) {
    return new Pose3d(
        p.getX(),
        p.getY(),
        Constants.CoralDetectorConstants.DISPLAY_Z_OFFSEET,
        new Rotation3d(0, 0, p.getRotation().getRadians()));
  }

  /**
   * @param dist the distance in meters the coral is from the lens of the cam along the floor
   * @param x the rotation in rads horizontally that the coal is from the center of the cam screen
   * @return the Translation2d from the robot to the coal
   */
  private Transform2d getCamToCoralTf(double dist, double angle) {
    // Validated 4/13 (CJT) correct other than a possible FOV error.
    Translation2d camera_to_coral_translation =
        new Translation2d(dist, Rotation2d.fromRadians(angle));
    return new Transform2d(camera_to_coral_translation, Rotation2d.kZero);
  }

  public Pose2d getCoralPose2d() {
    return io_.coral_pose_;
  }

  public boolean isValid() {
    return io_.target_valid_;
  }

  private void updateValidity() {
    boolean check =
        io_.can_see_coral_
            && (io_.target_distance_ < Constants.CoralDetectorConstants.DETECTION_DISTANCE_LIMIT);
    io_.target_valid_ =
        validity_debouncer_falling_.calculate(validity_debouncer_rising_.calculate(check));
  }

  public class CoralDetectorPeriodicIo implements Logged {
    @Log.File double target_x_ = 0.0; // Cross robot angle
    @Log.File double target_y_ = 0.0; // down track angle
    @Log.File boolean target_valid_ = false;
    @Log.File double target_distance_ = 0.0;
    @Log.File boolean can_see_coral_ = false;
    @Log.File Pose2d coral_pose_ = Pose2d.kZero;
    @Log.File Rotation2d attack_angle_ = Rotation2d.kZero;
    @Log.File Transform2d cam_to_coral_transform_ = Transform2d.kZero;
    @Log.File Transform2d bot_to_coral_transform_ = Transform2d.kZero;
  }

  public Logged getLoggingObject() {
    return io_;
  }
}
