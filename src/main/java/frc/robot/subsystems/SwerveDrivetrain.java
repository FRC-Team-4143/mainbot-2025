/*
 * Copyright (C) Cross The Road Electronics.  All rights reserved.
 * License information can be found in CTRE_LICENSE.txt
 * For support and suggestions contact support@ctr-electronics.com or file
 * an issue tracker at https://github.com/CrossTheRoadElec/Phoenix-Releases
 */
package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Radians;

import choreo.trajectory.SwerveSample;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.mw_lib.proxy_server.ChassisProxyServer;
import frc.mw_lib.subsystem.Subsystem;
import frc.mw_lib.swerve.*;
import frc.mw_lib.swerve.SwerveRequest.ForwardReference;
import frc.mw_lib.swerve.SwerveRequest.SwerveControlRequestParameters;
import frc.mw_lib.util.Util;
import frc.robot.Constants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.OI;
import java.util.function.Supplier;
import monologue.Annotations.Log;
import monologue.Logged;

/**
 * Swerve Drive class utilizing CTR Electronics' Phoenix 6 API.
 *
 * <p>This class handles the kinematics, configuration, and odometry of a swerve drive utilizing CTR
 * Electronics devices. We recommend that users use the Swerve Mechanism Generator in Tuner X to
 * create a template project that demonstrates how to use this class.
 *
 * <p>This class will construct the hardware devices internally, so the user only specifies the
 * constants (IDs, PID gains, gear ratios, etc). Getters for these hardware devices are available.
 *
 * <p>If using the generator, the order in which modules are constructed is Front Left, Front Right,
 * Back Left, Back Right. This means if you need the Back Left module, call {@code getModule(2);} to
 * get the 3rd index (0-indexed) module, corresponding to the Back Left module.
 */
public class SwerveDrivetrain extends Subsystem {
  private static SwerveDrivetrain instance_;

  public static SwerveDrivetrain getInstance() {
    if (instance_ == null) {
      instance_ =
          new SwerveDrivetrain(
              DrivetrainConstants.FL_MODULE_CONSTANTS,
              DrivetrainConstants.FR_MODULE_CONSTANTS,
              DrivetrainConstants.BL_MODULE_CONSTANTS,
              DrivetrainConstants.BR_MODULE_CONSTANTS);
    }
    return instance_;
  }

  // Subsystem data class
  private SwerveDrivetrainPeriodicIo io_;

  // Drive Mode Selections
  public enum DriveMode {
    ROBOT_CENTRIC,
    FIELD_CENTRIC,
    TARGET_FACING,
    TRAJECTORY,
    TRACTOR_BEAM,
    IDLE,
    PROFILE
  }

  // Robot Hardware
  private final Pigeon2 pigeon_imu_;
  private final SwerveModule[] swerve_modules_;

  // Drivetrain config
  final SwerveDriveKinematics kinematics_;
  private final Translation2d[] module_locations_;

  // Drive requests
  private SwerveRequest.FieldCentric auto_request_;
  private SwerveRequest.FieldCentric field_centric_;
  private SwerveRequest.RobotCentric robot_centric_;
  private SwerveRequest.FieldCentricFacingAngle field_centric_target_facing_;

  private SwerveRequest request_to_apply_;
  private SwerveControlRequestParameters request_parameters_;

  /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
  public final Rotation2d BLUE_ALLIANCE_HEADING = Rotation2d.fromDegrees(0);
  /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
  public final Rotation2d RED_ALLIANCE_HEADING = Rotation2d.fromDegrees(180);

  // NT publishers
  private StructArrayPublisher<SwerveModuleState> current_state_pub_,
      requested_state_pub_,
      current_state_proxy_pub_;

  // Field Widget for displaying poses
  private Field2d field_ = new Field2d();

  // PID Controllers
  private final PIDController x_traj_controller_;
  private final PIDController y_traj_controller_;
  private final PIDController heading_traj_controller_;
  private final PIDController x_pose_controller_;
  private final PIDController y_pose_controller_;
  private final PIDController heading_pose_controller_;

  /**
   * Constructs a SwerveDrivetrain using the specified constants.
   *
   * <p>This constructs the underlying hardware devices, so user should not construct the devices
   * themselves. If they need the devices, they can access them through getters in the classes.
   *
   * @param modules Constants for each specific module
   */
  public SwerveDrivetrain(SwerveModuleConstants... modules) {

    // make new io instance
    io_ = new SwerveDrivetrainPeriodicIo();

    // configure chassis server for comms
    ChassisProxyServer.configureServer();

    // Setup the Pigeon IMU
    pigeon_imu_ =
        new Pigeon2(DrivetrainConstants.PIGEON2_ID, DrivetrainConstants.MODULE_CANBUS_NAME[0]);
    pigeon_imu_.optimizeBusUtilization();

    // PID Controllers
    x_traj_controller_ = Constants.DrivetrainConstants.X_TRAJECTORY_TRANSLATION;
    y_traj_controller_ = Constants.DrivetrainConstants.Y_TRAJECTORY_TRANSLATION;
    heading_traj_controller_ = Constants.DrivetrainConstants.TRAJECTORY_HEADING;
    heading_traj_controller_.enableContinuousInput(-Math.PI, Math.PI);
    x_pose_controller_ = Constants.DrivetrainConstants.X_POSE_TRANSLATION;
    y_pose_controller_ = Constants.DrivetrainConstants.Y_POSE_TRANSLATION;
    heading_pose_controller_ = Constants.DrivetrainConstants.POSE_HEADING;
    heading_pose_controller_.enableContinuousInput(-Math.PI, Math.PI);

    SmartDashboard.putData(x_traj_controller_);
    SmartDashboard.putData(y_traj_controller_);
    SmartDashboard.putData(heading_traj_controller_);

    // Begin configuring swerve modules
    module_locations_ = new Translation2d[modules.length];
    swerve_modules_ = new SwerveModule[modules.length];
    io_.module_positions_ = new SwerveModulePosition[modules.length];
    io_.current_module_states_ = new SwerveModuleState[modules.length];
    io_.requested_module_states_ = new SwerveModuleState[modules.length];

    // Construct the swerve modules
    for (int i = 0; i < modules.length; i++) {
      SwerveModuleConstants module = modules[i];
      swerve_modules_[i] = new SwerveModule(module, DrivetrainConstants.MODULE_CANBUS_NAME[i]);
      module_locations_[i] = new Translation2d(module.LocationX, module.LocationY);
      io_.module_positions_[i] = swerve_modules_[i].getPosition(true);
      io_.current_module_states_[i] = swerve_modules_[i].getCurrentState();
      io_.requested_module_states_[i] = swerve_modules_[i].getTargetState();
    }
    kinematics_ = new SwerveDriveKinematics(module_locations_);

    // Drive mode requests
    field_centric_ =
        new SwerveRequest.FieldCentric()
            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
            .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagic)
            .withDeadband(DrivetrainConstants.MAX_DRIVE_SPEED * 0.01)
            .withRotationalDeadband(DrivetrainConstants.MAX_DRIVE_ANGULAR_RATE * 0.01);
    field_centric_.ForwardReference = ForwardReference.OperatorPerspective;
    robot_centric_ =
        new SwerveRequest.RobotCentric()
            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
            .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagic)
            .withDeadband(DrivetrainConstants.MAX_DRIVE_SPEED * 0.01)
            .withRotationalDeadband(DrivetrainConstants.MAX_DRIVE_ANGULAR_RATE * 0.01);
    field_centric_target_facing_ =
        new SwerveRequest.FieldCentricFacingAngle()
            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
            .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagic)
            .withDeadband(DrivetrainConstants.MAX_DRIVE_SPEED * 0.01)
            .withRotationalDeadband(DrivetrainConstants.MAX_DRIVE_ANGULAR_RATE * 0.01);
    field_centric_target_facing_.ForwardReference = ForwardReference.OperatorPerspective;
    auto_request_ =
        new SwerveRequest.FieldCentric()
            .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
            .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagic);
    auto_request_.ForwardReference = ForwardReference.RedAlliance;
    request_parameters_ = new SwerveControlRequestParameters();
    request_to_apply_ = new SwerveRequest.Idle();

    // NT Publishers
    requested_state_pub_ =
        NetworkTableInstance.getDefault()
            .getStructArrayTopic("module_states/requested", SwerveModuleState.struct)
            .publish();
    current_state_pub_ =
        NetworkTableInstance.getDefault()
            .getStructArrayTopic("module_states/current", SwerveModuleState.struct)
            .publish();
    current_state_proxy_pub_ =
        NetworkTableInstance.getDefault()
            .getStructArrayTopic("module_states/current_proxy", SwerveModuleState.struct)
            .publish();
  }

  @Override
  public void reset() {
    // 4 signals for each module + 2 for Pigeon2
    for (int i = 0; i < swerve_modules_.length; ++i) {
      BaseStatusSignal.setUpdateFrequencyForAll(100, swerve_modules_[i].getSignals());
      swerve_modules_[i].optimizeCan();
      swerve_modules_[i].resetToAbsolute();
    }
    BaseStatusSignal[] imuSignals = {pigeon_imu_.getYaw()};
    BaseStatusSignal.setUpdateFrequencyForAll(100, imuSignals);
    pigeon_imu_.optimizeBusUtilization();
  }

  @Override
  public void readPeriodicInputs(double timestamp) {
    for (int i = 0; i < swerve_modules_.length; ++i) {
      io_.module_positions_[i] = swerve_modules_[i].getPosition(true);
      io_.current_module_states_[i] = swerve_modules_[i].getCurrentState();
      io_.requested_module_states_[i] = swerve_modules_[i].getTargetState();
    }
    io_.driver_joystick_leftX_ = OI.getDriverJoystickLeftX();
    io_.driver_joystick_leftY_ = OI.getDriverJoystickLeftY();
    io_.driver_joystick_rightX_ = OI.getDriverJoystickRightX();

    io_.robot_yaw_ =
        Rotation2d.fromRadians(MathUtil.angleModulus(pigeon_imu_.getYaw().getValue().in(Radians)));

    io_.chassis_speeds_ = kinematics_.toChassisSpeeds(io_.current_module_states_);
    io_.field_relative_chassis_speed_ =
        ChassisSpeeds.fromRobotRelativeSpeeds(io_.chassis_speeds_, io_.robot_yaw_);

    io_.chassis_speed_magnitude_ =
        Math.sqrt(
            (io_.chassis_speeds_.vxMetersPerSecond * io_.chassis_speeds_.vxMetersPerSecond)
                + (io_.chassis_speeds_.vyMetersPerSecond * io_.chassis_speeds_.vyMetersPerSecond));

    io_.current_pose_ = PoseEstimator.getInstance().getFieldPose();

    // recieve new chassis info
    ChassisProxyServer.updateData();
  }

  @Override
  public void updateLogic(double timestamp) {
    request_parameters_.currentPose = new Pose2d(0, 0, io_.robot_yaw_);
    switch (io_.drive_mode_) {
      case ROBOT_CENTRIC:
        setControl(
            robot_centric_
                // Drive forward with negative Y (forward)
                .withVelocityX(-io_.driver_joystick_leftY_ * DrivetrainConstants.MAX_DRIVE_SPEED)
                // Drive left with negative X (left)
                .withVelocityY(-io_.driver_joystick_leftX_ * DrivetrainConstants.MAX_DRIVE_SPEED)
                // Drive counterclockwise with negative X (left)
                .withRotationalRate(
                    -io_.driver_joystick_rightX_ * DrivetrainConstants.MAX_DRIVE_ANGULAR_RATE));
        break;
      case FIELD_CENTRIC:
        setControl(
            field_centric_
                // Drive forward with negative Y (forward)
                .withVelocityX(-io_.driver_joystick_leftY_ * DrivetrainConstants.MAX_DRIVE_SPEED)
                // Drive left with negative X (left)
                .withVelocityY(-io_.driver_joystick_leftX_ * DrivetrainConstants.MAX_DRIVE_SPEED)
                // Drive counterclockwise with negative X (left)
                .withRotationalRate(
                    -io_.driver_joystick_rightX_ * DrivetrainConstants.MAX_DRIVE_ANGULAR_RATE));
        break;
      case TARGET_FACING:
        setControl(
            field_centric_target_facing_
                // Drive forward with negative Y (forward)
                .withVelocityX(
                    Util.clamp(
                        -io_.driver_joystick_leftY_ * DrivetrainConstants.MAX_DRIVE_SPEED,
                        DrivetrainConstants.MAX_TARGET_SPEED))
                // Drive left with negative X (left)
                .withVelocityY(
                    Util.clamp(
                        -io_.driver_joystick_leftX_ * DrivetrainConstants.MAX_DRIVE_SPEED,
                        DrivetrainConstants.MAX_TARGET_SPEED))
                // Set Robots target rotation
                .withTargetDirection(io_.target_rotation_)
                // Use current robot rotation
                .useGyroForRotation(false));
        break;
      case TRAJECTORY:
        this.setControl(
            auto_request_
                .withVelocityX(
                    io_.target_sample_.vx
                        + x_traj_controller_.calculate(
                            io_.current_pose_.getX(), io_.target_sample_.x))
                .withVelocityY(
                    io_.target_sample_.vy
                        + y_traj_controller_.calculate(
                            io_.current_pose_.getY(), io_.target_sample_.y))
                .withRotationalRate(
                    (io_.target_sample_.omega)
                        + heading_traj_controller_.calculate(
                            io_.current_pose_.getRotation().getRadians(),
                            io_.target_sample_.heading)));
        request_parameters_.currentPose = io_.target_pose_;
        break;
      case TRACTOR_BEAM:
        io_.tractor_beam_scaling_factor_ =
            Math.sqrt(
                Math.pow(io_.driver_joystick_leftX_, 2)
                    + Math.pow(io_.driver_joystick_leftY_, 2)
                    + Math.pow(io_.driver_joystick_rightX_, 2));
        this.setControl(
            auto_request_
                .withVelocityX(
                    x_pose_controller_.calculate(io_.current_pose_.getX(), io_.target_pose_.getX())
                        * io_.tractor_beam_scaling_factor_)
                .withVelocityY(
                    y_pose_controller_.calculate(io_.current_pose_.getY(), io_.target_pose_.getY())
                        * io_.tractor_beam_scaling_factor_)
                .withRotationalRate(
                    heading_pose_controller_.calculate(
                            io_.current_pose_.getRotation().getRadians(),
                            io_.target_pose_.getRotation().getRadians())
                        * io_.tractor_beam_scaling_factor_));
        request_parameters_.currentPose = io_.target_pose_;
        break;
      case IDLE:
        setControl(new SwerveRequest.Idle());
        break;
      default:
        break;
    }

    /* And now that we've got the new odometry, update the controls */

    request_parameters_.kinematics = kinematics_;
    request_parameters_.swervePositions = module_locations_;
    request_parameters_.updatePeriod = timestamp - request_parameters_.timestamp;
    request_parameters_.timestamp = timestamp;
    request_parameters_.operatorForwardDirection = io_.drivers_station_perspective_;
  }

  @Override
  public void writePeriodicOutputs(double timestamp) {
    request_to_apply_.apply(request_parameters_, swerve_modules_);
  }

  @Override
  public void outputTelemetry(double timestamp) {
    current_state_pub_.set(io_.current_module_states_);
    requested_state_pub_.set(io_.requested_module_states_);
    current_state_proxy_pub_.set(ChassisProxyServer.getModuleStates());
    SmartDashboard.putString("Debug/Swerve/Mode", io_.drive_mode_.toString());
    SmartDashboard.putNumber(
        "Debug/Swerve/Rotation Control/Target Rotation", io_.target_rotation_.getDegrees());
    SmartDashboard.putNumber(
        "Debug/Swerve/Rotation Control/Current Yaw", io_.robot_yaw_.getDegrees());
    SmartDashboard.putNumber(
        "Debug/Chassis Speed/Omega", io_.chassis_speeds_.omegaRadiansPerSecond);
    SmartDashboard.putNumber("chassis_speed_magnitude_", io_.chassis_speed_magnitude_);
    SmartDashboard.putNumber(
        "Debug/Swerve/Driver Prespective", io_.drivers_station_perspective_.getDegrees());
    SmartDashboard.putNumber("Debug/Swerve/Chassis Speed/X", io_.chassis_speeds_.vxMetersPerSecond);
    SmartDashboard.putNumber("Debug/Swerve/Chassis Speed/Y", io_.chassis_speeds_.vyMetersPerSecond);
    SmartDashboard.putNumber(
        "Debug/Swerve/Chassis Speed/Omega", io_.chassis_speeds_.omegaRadiansPerSecond);

    Twist2d chassis_proxy_twist = ChassisProxyServer.getTwist();
    SmartDashboard.putNumber("Debug/Swerve/Chassis Proxy Speed/X", chassis_proxy_twist.dx);
    SmartDashboard.putNumber("Debug/Swerve/Chassis Proxy Speed/Y", chassis_proxy_twist.dy);
    SmartDashboard.putNumber("Debug/Swerve/Chassis Proxy Speed/Omega", chassis_proxy_twist.dtheta);
    field_.setRobotPose(ChassisProxyServer.getPose());
    SmartDashboard.putData("Chassis Proxy Pose", field_);

    SmartDashboard.putString("drive mode", io_.drive_mode_.toString());

    SmartDashboard.putNumber("Encoder:0", swerve_modules_[0].getEncoderValue() * 360);
    SmartDashboard.putNumber("Encoder:1", swerve_modules_[1].getEncoderValue() * 360);
    SmartDashboard.putNumber("Encoder:2", swerve_modules_[2].getEncoderValue() * 360);
    SmartDashboard.putNumber("Encoder:3", swerve_modules_[3].getEncoderValue() * 360);

    SmartDashboard.putString("requestType", request_to_apply_.toString());
  }

  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return Commands.run(() -> this.setControl(requestSupplier.get()));
  }

  /**
   * Takes the current orientation of the robot and makes it X forward for field-relative maneuvers.
   */
  public void seedFieldRelative(Rotation2d seed_rotation) {
    pigeon_imu_.setYaw(seed_rotation.getDegrees());
    io_.robot_yaw_ =
        Rotation2d.fromRadians(MathUtil.angleModulus(pigeon_imu_.getYaw().getValue().in(Radians)));
  }

  /**
   * Takes the current orientation of the robot and makes it X forward for field-relative maneuvers.
   */
  public Command seedFieldRelativeCommand() {
    return Commands.runOnce(() -> seedFieldRelative(io_.drivers_station_perspective_));
  }

  /**
   * Applies the specified control request to this swerve drivetrain.
   *
   * @param request Request to apply
   */
  public void setControl(SwerveRequest request) {
    request_to_apply_ = request;
  }

  /**
   * Applies the specified control request to this swerve drivetrain.
   *
   * @param request Request to apply
   */
  public void applyChassisSpeeds(ChassisSpeeds speeds) {
    this.setControl(new SwerveRequest.ApplyChassisSpeeds().withSpeeds(speeds));
  }

  /**
   * Zero's this swerve drive's odometry entirely. This will zero the entire odometry, and place the
   * robot at 0,0
   */
  public void tareEverything() {
    for (int i = 0; i < swerve_modules_.length; ++i) {
      swerve_modules_[i].resetPosition();
      swerve_modules_[i].setWheelOffsets();
      io_.module_positions_[i] = swerve_modules_[i].getPosition(true);
    }
  }

  /** Gets the raw value from the Robot IMU */
  public Rotation2d getImuYaw() {
    return io_.robot_yaw_;
  }

  /**
   * Returns the module locations in reference to the center of the robot as an array [FrontLeft,
   * FrontRight, BackLeft, BackRight]
   */
  public SwerveModulePosition[] getModulePositions() {
    return io_.module_positions_;
  }

  /**
   * Returns the module states of the swerve drive as an array [FrontLeft, FrontRight, BackLeft,
   * BackRight]
   */
  public SwerveModuleState[] getModuleStates() {
    return io_.current_module_states_;
  }

  /**
   * updates the mode flag thats changes what request is applied to the drive train
   *
   * @param mode requewt drive mode
   */
  public void setDriveMode(DriveMode mode) {
    io_.drive_mode_ = mode;
  }

  /** Toggles between FIELD_CENTRIC and ROBOT_CENTRIC drive modes */
  public Command toggleFieldCentric() {
    return Commands.runOnce(
        () -> {
          io_.drive_mode_ =
              (io_.drive_mode_ == DriveMode.FIELD_CENTRIC)
                  ? DriveMode.ROBOT_CENTRIC
                  : DriveMode.FIELD_CENTRIC;
        });
  }

  /**
   * Sets the drivers perspective for field relative requests
   *
   * @param prespective rotation to set as forward perspective
   */
  public void setDriverPerspective(Rotation2d prespective) {
    io_.drivers_station_perspective_ = prespective;
  }

  /**
   * Updates the internal target for the robot to follow and begins TRAJECTORY mode
   *
   * @param sample swerve drive sample that includes target twist and pose
   */
  public void setTargetSample(SwerveSample sample) {
    io_.drive_mode_ = DriveMode.TRAJECTORY;
    io_.target_sample_ = sample;
    io_.target_pose_ = new Pose2d(sample.x, sample.y, new Rotation2d(sample.heading));
  }

  /**
   * Updates the internal target for the robot to reach and begins TRACTOR_BEAM mode
   *
   * @param target_pose target pose for the robot to reach
   */
  public void setTargetPose(Pose2d target_pose) {
    io_.drive_mode_ = DriveMode.TRACTOR_BEAM;
    io_.target_pose_ = target_pose;
  }

  /**
   * Updates the internal target for the robot to point at and begins TARGET_FACING mode
   *
   * @param target_angle
   */
  public void setTargetRotation(Rotation2d target_angle) {
    io_.drive_mode_ = DriveMode.TARGET_FACING;
    io_.target_rotation_ = target_angle;
  }

  /**
   * Plain-Old-Data class holding the state of the swerve drivetrain. This encapsulates most data
   * that is relevant for telemetry or decision-making from the Swerve Drive.
   */
  public class SwerveDrivetrainPeriodicIo implements Logged {
    @Log.File public SwerveModuleState[] current_module_states_, requested_module_states_;
    @Log.File public SwerveModulePosition[] module_positions_;
    @Log.File public double driver_joystick_leftX_ = 0.0;
    @Log.File public double driver_joystick_leftY_ = 0.0;
    @Log.File public double driver_joystick_rightX_ = 0.0;
    @Log.File public Rotation2d robot_yaw_ = new Rotation2d();
    @Log.File public ChassisSpeeds chassis_speeds_ = new ChassisSpeeds();
    @Log.File public ChassisSpeeds field_relative_chassis_speed_ = new ChassisSpeeds();
    @Log.File public double chassis_speed_magnitude_ = 0.0;
    @Log.File public Rotation2d target_rotation_ = new Rotation2d();
    @Log.File public DriveMode drive_mode_ = DriveMode.IDLE;
    @Log.File public double driver_POVx = 0.0;
    @Log.File public double driver_POVy = 0.0;
    @Log.File public Rotation2d drivers_station_perspective_ = new Rotation2d();
    @Log.File public double tractor_beam_scaling_factor_ = 0.0;
    @Log.File public Pose2d current_pose_ = new Pose2d();
    @Log.File public Pose2d target_pose_ = new Pose2d();

    @Log.File
    public SwerveSample target_sample_ = new SwerveSample(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, null, null);
  }

  @Override
  public Logged getLoggingObject() {
    return io_;
  }
}
