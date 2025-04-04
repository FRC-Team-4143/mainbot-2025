// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.xml.crypto.dsig.spec.TransformParameterSpec;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.mw_lib.subsystem.Subsystem;
import frc.robot.Constants.CoralDetectorConstants;
import monologue.Annotations.Log;
import monologue.Logged;

public class CoralDetector extends Subsystem {

  // Singleton pattern
  private static CoralDetector coral_detector_instance_ = null;

  private NetworkTable table_;
  private NetworkTableEntry table_entry_tx_, table_entry_ty_, table_entry_ta_, table_entry_tv_;

  private Debouncer rising_debouncer_;
  private Debouncer falling_debouncer_;

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

    table_ = NetworkTableInstance.getDefault().getTable("limelight");
    table_entry_ty_ = table_.getEntry("ty");
    table_entry_tx_ = table_.getEntry("tx");
    table_entry_ta_ = table_.getEntry("ta");
    table_entry_tv_ = table_.getEntry("tv");

    rising_debouncer_ = new Debouncer(CoralDetectorConstants.DETECT_FALLING,
        Debouncer.DebounceType.kFalling);
    falling_debouncer_ = new Debouncer(CoralDetectorConstants.DETECT_RISING,
        Debouncer.DebounceType.kRising);

    // Call reset last in subsystem configuration
    reset();
  }

  /**
   * This function should be logic and code to fully reset your subsystem. This is
   * called during
   * initialization, and should handle I/O configuration and initializing data
   * members.
   */
  @Override
  public void reset() {
  }

  /**
   * Inside this function, all of the SENSORS should be read into variables stored
   * in the PeriodicIO
   * class defined below. There should be no calls to output to actuators, or any
   * logic within this
   * function.
   */
  @Override
  public void readPeriodicInputs(double timestamp) {
    io_.target_x_ = Math.toRadians(table_entry_tx_.getDouble(0));
    io_.target_y_ = Math.toRadians(table_entry_ty_.getDouble(0));
    io_.target_accuracy_ = table_entry_ta_.getDouble(0.0);
    io_.target_valid_ = table_entry_tv_.getInteger(0);
  }

  /**
   * Inside this function, all of the LOGIC should compute updates to output
   * variables in the
   * PeriodicIO class defined below. There should be no calls to read from sensors
   * or write to
   * actuators in this function.
   */
  @Override
  public void updateLogic(double timestamp) {
    io_.target_distance_ = calculateDist(io_.target_y_);
    io_.can_see_coral_ = rising_debouncer_.calculate(io_.target_valid_ == 1)
        && io_.target_distance_ <= CoralDetectorConstants.DETECTION_DISTANCE_LIMIT;
    if (io_.can_see_coral_) {
      io_.coral_pose_ = caluclateCoralPose2d();
    }
  }

  /**
   * Inside this function actuator OUTPUTS should be updated from data contained
   * in the PeriodicIO
   * class defined below. There should be little to no logic contained within this
   * function, and no
   * sensors should be read.
   */
  @Override
  public void writePeriodicOutputs(double timestamp) {
  }

  /**
   * Inside this function telemetry should be output to smartdashboard. The data
   * should be collected
   * out of the PeriodicIO class instance defined below. There should be no sensor
   * information read
   * in this function nor any outputs made to actuators within this function. Only
   * publish to
   * smartdashboard here.
   */
  @Override
  public void outputTelemetry(double timestamp) {
  }

  /**
   * @param y the rotation in rads verticaly that the coal is from the center of
   *          the cam
   *          screen
   * @return the distance in meaters the coral is from the lens of the cam along
   *         the floor
   */
  public double calculateDist(double y) {
    double angleToGoal = CoralDetectorConstants.CORAL_CAM_CONSTANTS.camera_transform.getRotation().getY() + y;
    return (CoralDetectorConstants.COARL_HEIGHT_METERS
        - CoralDetectorConstants.CORAL_CAM_CONSTANTS.camera_transform.getZ())
        / Math.tan(angleToGoal);
  }

  /**
   * @param dist the distance in meaters the coral is from the lens of the cam
   *             along the floor
   * @param x    the rotation in rads horizontaly that the coal is from the center
   *             of the cam
   *             screen
   * @return the Translation2d from the robot to the coal
   */
  private Transform2d caluclateCoralTransform2d(double dist, double x) {
    Transform3d camTransform = CoralDetectorConstants.CORAL_CAM_CONSTANTS.camera_transform;
    Transform2d coralToCam = new Transform2d(new Translation2d(dist, Rotation2d.fromRadians(x)), Rotation2d.kZero);
    Transform2d botToCam = new Transform2d(camTransform.getX(), camTransform.getY(),
        camTransform.getRotation().toRotation2d());
    Transform2d finalTransform = coralToCam.plus(botToCam);
    return finalTransform;
  }

  /**
   * @return the Pose2d of the coal on the feild
   */
  private Pose2d caluclateCoralPose2d() {
    return PoseEstimator.getInstance().getRobotPose()
        .transformBy(caluclateCoralTransform2d(io_.target_distance_, io_.target_x_));
  }

  public Pose2d getCoralPose2d() {
    return io_.coral_pose_;
  }

  public Transform2d getCoralTransform2d() {
    return caluclateCoralTransform2d(io_.target_distance_, io_.target_x_);
  }

  public class CoralDetectorPeriodicIo implements Logged {
    @Log.File
    double target_x_ = 0.0;
    @Log.File
    double target_y_ = 0.0;
    @Log.File
    double target_accuracy_ = 0.0;
    @Log.File
    long target_valid_ = 0; // not an int since the network tables only return longs
    @Log.File
    double target_distance_ = 0.0;
    @Log.File
    boolean can_see_coral_ = false;
    @Log.File
    Pose2d coral_pose_ = new Pose2d();
  }

  public Logged getLoggingObject() {
    return io_;
  }
}
