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
import java.util.Optional;
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
  private static SwerveDrivetrain instance;

  public static SwerveDrivetrain getInstance() {
    if (instance == null) {
      instance =
          new SwerveDrivetrain(
              DrivetrainConstants.FL_MODULE_CONSTANTS,
              DrivetrainConstants.FR_MODULE_CONSTANTS,
              DrivetrainConstants.BL_MODULE_CONSTANTS,
              DrivetrainConstants.BR_MODULE_CONSTANTS);
    }
    return instance;
  }

  // Subsystem data class
  private SwerveDriverainPeriodicIo io_;

  // Drive Mode Selections
  public enum DriveMode {
    ROBOT_CENTRIC,
    FIELD_CENTRIC,
    TARGET,
    AUTONOMOUS,
    AUTONOMOUS_TARGET,
    IDLE
  }

  // Robot Hardware
  private final Pigeon2 pigeon_imu;
  private final SwerveModule[] swerve_modules;

  // Drivetrain config
  final SwerveDriveKinematics kinematics;
  private final Translation2d[] module_locations;

  // Drive requests
  private SwerveRequest.FieldCentric auto_request;
  private SwerveRequest.FieldCentric field_centric;
  private SwerveRequest.RobotCentric robot_centric;
  private SwerveRequest.FieldCentricFacingAngle target_facing;

  private SwerveRequest request_to_apply;
  private SwerveControlRequestParameters request_parameters;

  /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
  public final Rotation2d blueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
  /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
  public final Rotation2d redAlliancePerspectiveRotation = Rotation2d.fromDegrees(180);
  /* Keep track if we've ever applied the operator perspective before or not */
  public boolean hasAppliedOperatorPerspective = false;

  // NT publishers
  private StructArrayPublisher<SwerveModuleState> current_state_pub,
      requested_state_pub,
      current_state_proxy_pub;
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
    io_ = new SwerveDriverainPeriodicIo();

    // configure chassis server for comms
    ChassisProxyServer.configureServer();

    // Setup the Pigeon IMU
    pigeon_imu =
        new Pigeon2(DrivetrainConstants.PIGEON2_ID, DrivetrainConstants.MODULE_CANBUS_NAME[0]);
    pigeon_imu.optimizeBusUtilization();

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
    module_locations = new Translation2d[modules.length];
    swerve_modules = new SwerveModule[modules.length];
    io_.module_positions = new SwerveModulePosition[modules.length];
    io_.current_module_states_ = new SwerveModuleState[modules.length];
    io_.requested_module_states_ = new SwerveModuleState[modules.length];

    // Construct the swerve modules
    for (int i = 0; i < modules.length; i++) {
      SwerveModuleConstants module = modules[i];
      swerve_modules[i] = new SwerveModule(module, DrivetrainConstants.MODULE_CANBUS_NAME[i]);
      module_locations[i] = new Translation2d(module.LocationX, module.LocationY);
      io_.module_positions[i] = swerve_modules[i].getPosition(true);
      io_.current_module_states_[i] = swerve_modules[i].getCurrentState();
      io_.requested_module_states_[i] = swerve_modules[i].getTargetState();
    }
    kinematics = new SwerveDriveKinematics(module_locations);

    // Drive mode requests
    field_centric =
        new SwerveRequest.FieldCentric()
            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
            .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagic)
            .withDeadband(DrivetrainConstants.MAX_DRIVE_SPEED * 0.01)
            .withRotationalDeadband(DrivetrainConstants.MAX_DRIVE_ANGULAR_RATE * 0.01);
    field_centric.ForwardReference = ForwardReference.OperatorPerspective;
    robot_centric =
        new SwerveRequest.RobotCentric()
            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
            .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagic)
            .withDeadband(DrivetrainConstants.MAX_DRIVE_SPEED * 0.01)
            .withRotationalDeadband(DrivetrainConstants.MAX_DRIVE_ANGULAR_RATE * 0.01);
    target_facing = 
        new SwerveRequest.FieldCentricFacingAngle()
            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
            .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagic)
            .withDeadband(DrivetrainConstants.MAX_DRIVE_SPEED * 0.01)
            .withRotationalDeadband(DrivetrainConstants.MAX_DRIVE_ANGULAR_RATE * 0.01);
    target_facing.ForwardReference = ForwardReference.OperatorPerspective;
    auto_request =
        new SwerveRequest.FieldCentric()
            .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
            .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagic);
    auto_request.ForwardReference = ForwardReference.RedAlliance;
    request_parameters = new SwerveControlRequestParameters();
    request_to_apply = new SwerveRequest.Idle();

    // NT Publishers
    requested_state_pub =
        NetworkTableInstance.getDefault()
            .getStructArrayTopic("module_states/requested", SwerveModuleState.struct)
            .publish();
    current_state_pub =
        NetworkTableInstance.getDefault()
            .getStructArrayTopic("module_states/current", SwerveModuleState.struct)
            .publish();
    current_state_proxy_pub =
        NetworkTableInstance.getDefault()
            .getStructArrayTopic("module_states/current_proxy", SwerveModuleState.struct)
            .publish();
  }

  @Override
  public void reset() {
    // 4 signals for each module + 2 for Pigeon2
    for (int i = 0; i < swerve_modules.length; ++i) {
      BaseStatusSignal.setUpdateFrequencyForAll(100, swerve_modules[i].getSignals());
      swerve_modules[i].optimizeCan();
      swerve_modules[i].resetToAbsolute();
    }
    BaseStatusSignal[] imuSignals = {pigeon_imu.getYaw()};
    BaseStatusSignal.setUpdateFrequencyForAll(100, imuSignals);
    pigeon_imu.optimizeBusUtilization();
  }

  @Override
  public void readPeriodicInputs(double timestamp) {
    for (int i = 0; i < swerve_modules.length; ++i) {
      io_.module_positions[i] = swerve_modules[i].getPosition(true);
      io_.current_module_states_[i] = swerve_modules[i].getCurrentState();
      io_.requested_module_states_[i] = swerve_modules[i].getTargetState();
    }
    io_.driver_joystick_leftX_ = OI.getDriverJoystickLeftX();
    io_.driver_joystick_leftY_ = OI.getDriverJoystickLeftY();
    io_.driver_joystick_rightX_ = OI.getDriverJoystickRightX();

    io_.robot_yaw_ =
        Rotation2d.fromRadians(MathUtil.angleModulus(pigeon_imu.getYaw().getValue().in(Radians)));

    io_.chassis_speeds_ = kinematics.toChassisSpeeds(io_.current_module_states_);
    io_.field_relative_chassis_speed_ =
        ChassisSpeeds.fromRobotRelativeSpeeds(io_.chassis_speeds_, io_.robot_yaw_);

    io_.chassis_speed_magnitude_ =
        Math.sqrt(
            (io_.chassis_speeds_.vxMetersPerSecond * io_.chassis_speeds_.vxMetersPerSecond)
                + (io_.chassis_speeds_.vyMetersPerSecond * io_.chassis_speeds_.vyMetersPerSecond));

    // recieve new chassis info
    ChassisProxyServer.updateData();
  }

  @Override
  public void updateLogic(double timestamp) {
    switch (io_.drive_mode_) {
      case ROBOT_CENTRIC:
        setControl(
            robot_centric
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
            field_centric
                // Drive forward with negative Y (forward)
                .withVelocityX(-io_.driver_joystick_leftY_ * DrivetrainConstants.MAX_DRIVE_SPEED)
                // Drive left with negative X (left)
                .withVelocityY(-io_.driver_joystick_leftX_ * DrivetrainConstants.MAX_DRIVE_SPEED)
                // Drive counterclockwise with negative X (left)
                .withRotationalRate(
                    -io_.driver_joystick_rightX_ * DrivetrainConstants.MAX_DRIVE_ANGULAR_RATE));
        break;
      case TARGET:
        setControl(
            target_facing
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
                .useGyroForRotation(io_.is_locked_with_gyro));
        break;
      case IDLE:
        setControl(new SwerveRequest.Idle());
        break;
      case AUTONOMOUS:
      default:
        // yes these dont do anything for auto...
        break;
    }

    /* And now that we've got the new odometry, update the controls */
    request_parameters.currentPose = new Pose2d(0, 0, io_.robot_yaw_);
    request_parameters.kinematics = kinematics;
    request_parameters.swervePositions = module_locations;
    request_parameters.updatePeriod = timestamp - request_parameters.timestamp;
    request_parameters.timestamp = timestamp;
    request_parameters.operatorForwardDirection = io_.drivers_station_perspective_;
  }

  @Override
  public void writePeriodicOutputs(double timestamp) {
    request_to_apply.apply(request_parameters, swerve_modules);
  }

  @Override
  public void outputTelemetry(double timestamp) {
    current_state_pub.set(io_.current_module_states_);
    requested_state_pub.set(io_.requested_module_states_);
    current_state_proxy_pub.set(ChassisProxyServer.getModuleStates());

    SmartDashboard.putNumber("Rotation Control/Target Rotation", io_.target_rotation_.getDegrees());
    SmartDashboard.putNumber("Rotation Control/Current Yaw", io_.robot_yaw_.getDegrees());
    SmartDashboard.putNumber(
        "Debug/Driver Prespective", io_.drivers_station_perspective_.getDegrees());
    SmartDashboard.putNumber("Debug/Chassis Speed/X", io_.chassis_speeds_.vxMetersPerSecond);
    SmartDashboard.putNumber("Debug/Chassis Speed/Y", io_.chassis_speeds_.vyMetersPerSecond);
    SmartDashboard.putNumber(
        "Debug/Chassis Speed/Omega", io_.chassis_speeds_.omegaRadiansPerSecond);
    SmartDashboard.putNumber("chassis_speed_magnitude_", io_.chassis_speed_magnitude_);
    Twist2d chassis_proxy_twist = ChassisProxyServer.getTwist();
    SmartDashboard.putNumber("Debug/Chassis Proxy Speed/X", chassis_proxy_twist.dx);
    SmartDashboard.putNumber("Debug/Chassis Proxy Speed/Y", chassis_proxy_twist.dy);
    SmartDashboard.putNumber("Debug/Chassis Proxy Speed/Omega", chassis_proxy_twist.dtheta);
    field_.setRobotPose(ChassisProxyServer.getPose());
    SmartDashboard.putData("Chassis Proxy Pose", field_);

    SmartDashboard.putString("drive mode", io_.drive_mode_.toString());

    SmartDashboard.putNumber("Encoder:0", swerve_modules[0].getEncoderValue() * 360);
    SmartDashboard.putNumber("Encoder:1", swerve_modules[1].getEncoderValue() * 360);
    SmartDashboard.putNumber("Encoder:2", swerve_modules[2].getEncoderValue() * 360);
    SmartDashboard.putNumber("Encoder:3", swerve_modules[3].getEncoderValue() * 360);

    SmartDashboard.putString("requestType", request_to_apply.toString());
  }

  public Rotation2d getRobotRotation() {
    return io_.robot_yaw_;
  }

  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return Commands.run(() -> this.setControl(requestSupplier.get()));
  }

  /**
   * Takes the current orientation of the robot and makes it X forward for field-relative maneuvers.
   */
  public void seedFieldRelative(Rotation2d offset) {
    pigeon_imu.setYaw(offset.getDegrees());
    io_.robot_yaw_ =
        Rotation2d.fromRadians(MathUtil.angleModulus(pigeon_imu.getYaw().getValue().in(Radians)));
  }

  /**
   * Applies the specified control request to this swerve drivetrain.
   *
   * @param request Request to apply
   */
  public void setControl(SwerveRequest request) {
    request_to_apply = request;
  }

  public void applyChassisSpeeds(ChassisSpeeds speeds) {
    this.setControl(new SwerveRequest.ApplyChassisSpeeds().withSpeeds(speeds));
  }

  /**
   * Zero's this swerve drive's odometry entirely.
   *
   * <p>This will zero the entire odometry, and place the robot at 0,0
   */
  public void tareEverything() {
    for (int i = 0; i < swerve_modules.length; ++i) {
      swerve_modules[i].resetPosition();
      swerve_modules[i].setWheelOffsets();
      io_.module_positions[i] = swerve_modules[i].getPosition(true);
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
    return io_.module_positions;
  }

  /**
   * Returns the module states of the swerve drive as an array [FrontLeft, FrontRight, BackLeft,
   * BackRight]
   */
  public SwerveModuleState[] getModuleStates() {
    return io_.current_module_states_;
  }

  public void setTargetRotation(Rotation2d target_angle_) {
    io_.target_rotation_ = target_angle_;
  }

  public void setCrabRequest(Rotation2d target_angle_) {
    io_.driver_POVy = target_angle_.getCos();
    io_.driver_POVx = target_angle_.getSin();
  }

  public Optional<Rotation2d> getAutoTargetRotation() {
    if (io_.drive_mode_ == DriveMode.AUTONOMOUS_TARGET) {
      return Optional.of(io_.target_rotation_.rotateBy(io_.drivers_station_perspective_));
    }
    return Optional.empty();
  }

  /**
   * updates the mode flag thats changes what request is applied to the drive train
   *
   * @param mode drive to switch to [ROBOT_CENTRIC, FIELD_CENTRIC]
   */
  public void setDriveMode(DriveMode mode) {
    io_.drive_mode_ = mode;
  }

  public void toggleFieldCentric() {
    io_.drive_mode_ =
        (io_.drive_mode_ == DriveMode.FIELD_CENTRIC)
            ? DriveMode.ROBOT_CENTRIC
            : DriveMode.FIELD_CENTRIC;
  }

  public DriveMode getDriveMode() {
    return io_.drive_mode_;
  }

  public void setDriverPrespective(Rotation2d prespective) {
    io_.drivers_station_perspective_ = prespective;
  }

  public Rotation2d getDriverPrespective() {
    return io_.drivers_station_perspective_;
  }

  public void rotationTargetWithGyro(boolean state) {
    io_.is_locked_with_gyro = state;
  }

  public void followTrajectory(SwerveSample sample) {
    io_.drive_mode_ = DriveMode.AUTONOMOUS;
    Pose2d pose = PoseEstimator.getInstance().getFieldPose();
    this.setControl(
        auto_request
            .withVelocityX(sample.vx + x_traj_controller_.calculate(pose.getX(), sample.x))
            .withVelocityY(sample.vy + y_traj_controller_.calculate(pose.getY(), sample.y))
            .withRotationalRate(
                (sample.omega)
                    + heading_traj_controller_.calculate(
                        pose.getRotation().getRadians(), sample.heading)));
  }

  public void tractorBeam(Pose2d targetPose) {
    io_.tractor_beam_scaling_factor_ =
        Math.sqrt(
            Math.pow(io_.driver_joystick_leftX_, 2)
                + Math.pow(io_.driver_joystick_leftY_, 2)
                + Math.pow(io_.driver_joystick_rightX_, 2));

    Pose2d pose = PoseEstimator.getInstance().getFieldPose();
    this.setControl(
        auto_request
            .withVelocityX(
                x_pose_controller_.calculate(pose.getX(), targetPose.getX())
                    * io_.tractor_beam_scaling_factor_)
            .withVelocityY(
                y_pose_controller_.calculate(pose.getY(), targetPose.getY())
                    * io_.tractor_beam_scaling_factor_)
            .withRotationalRate(
                heading_pose_controller_.calculate(
                        pose.getRotation().getRadians(), targetPose.getRotation().getRadians())
                    * io_.tractor_beam_scaling_factor_));
  }

  /**
   * Plain-Old-Data class holding the state of the swerve drivetrain. This encapsulates most data
   * that is relevant for telemetry or decision-making from the Swerve Drive.
   */
  public class SwerveDriverainPeriodicIo implements Logged {
    @Log.File public SwerveModuleState[] current_module_states_, requested_module_states_;
    @Log.File public SwerveModulePosition[] module_positions;
    @Log.File public Rotation2d robot_yaw_ = new Rotation2d();
    @Log.File public double driver_joystick_leftX_ = 0.0;
    @Log.File public double driver_joystick_leftY_ = 0.0;
    @Log.File public double driver_joystick_rightX_ = 0.0;
    @Log.File public ChassisSpeeds chassis_speeds_ = new ChassisSpeeds();
    @Log.File public ChassisSpeeds field_relative_chassis_speed_ = new ChassisSpeeds();
    @Log.File public Rotation2d target_rotation_ = new Rotation2d();
    @Log.File public DriveMode drive_mode_ = DriveMode.IDLE;
    @Log.File public double driver_POVx = 0.0;
    @Log.File public double driver_POVy = 0.0;
    @Log.File public Rotation2d drivers_station_perspective_ = new Rotation2d();
    @Log.File public double chassis_speed_magnitude_;
    @Log.File public boolean is_locked_with_gyro = false;
    @Log.File public double tractor_beam_scaling_factor_ = 0.0;
  }

  @Override
  public Logged getLoggingObject() {
    return io_;
  }
}
