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
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.mw_lib.geometry.TightRope;
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
    TIGHT_ROPE,
    CRAWL,
    IDLE,
    PROFILE
  }

  public enum SpeedPresets {
    MAX_SPEED(1.0),
    THREE_FOURTHS_SPEED(0.75),
    HALF_SPEED(0.5),
    ONE_THIRD_SPEED(0.33);

    private SpeedPresets(double val) {
      speed_limit = val;
    }
    ;

    public final double speed_limit;
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
  private StructArrayPublisher<SwerveModuleState> current_state_pub_;
  private StructArrayPublisher<SwerveModuleState> requested_state_pub_;
  private StructPublisher<ChassisSpeeds> current_chassis_speeds_pub_;
  private StructPublisher<ChassisSpeeds> requested_chassis_speeds_pub_;
  private StructPublisher<Pose2d> tractorbeam_pose_;

  // PID Controllers
  private final PIDController x_traj_controller_;
  private final PIDController y_traj_controller_;
  private final PIDController heading_traj_controller_;
  private final PIDController x_pose_controller_;
  private final PIDController y_pose_controller_;
  private final PIDController heading_pose_controller_;

  private Debouncer stallingDebouncer;

  /**
   * Constructs a SwerveDrivetrain using the specified constants.
   *
   * <p>This constructs the underlying hardware devices, so user should not construct the devices
   * themselves. If they need the devices, they can access them through getters in the classes.
   *
   * @param modules Constants for each specific module
   */
  public SwerveDrivetrain(SwerveModuleConstants... modules) {

    // make a debouncer
    stallingDebouncer =
        new Debouncer(
            Constants.DrivetrainConstants.FAILING_TO_REACH_TARGET_DEBOUNCE_TIME,
            DebounceType.kRising);

    // make new io instance
    io_ = new SwerveDrivetrainPeriodicIo();

    // Setup the Pigeon IMU
    pigeon_imu_ =
        new Pigeon2(DrivetrainConstants.PIGEON2_ID, DrivetrainConstants.MODULE_CANBUS_NAME[0]);
    pigeon_imu_.optimizeBusUtilization();

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

    // PID Controllers
    x_traj_controller_ = Constants.DrivetrainConstants.X_TRAJECTORY_TRANSLATION;
    y_traj_controller_ = Constants.DrivetrainConstants.Y_TRAJECTORY_TRANSLATION;
    heading_traj_controller_ = Constants.DrivetrainConstants.TRAJECTORY_HEADING;
    heading_traj_controller_.enableContinuousInput(-Math.PI, Math.PI);
    x_pose_controller_ = Constants.DrivetrainConstants.X_POSE_TRANSLATION;
    y_pose_controller_ = Constants.DrivetrainConstants.Y_POSE_TRANSLATION;
    heading_pose_controller_ = Constants.DrivetrainConstants.POSE_HEADING;
    heading_pose_controller_.enableContinuousInput(-Math.PI, Math.PI);

    // Allow PID Configuration for Traj / Pose Control on the Dashboard
    SmartDashboard.putData("Tuning/Swerve/X Traj Controller", x_traj_controller_);
    SmartDashboard.putData("Tuning/Swerve/Y Traj Controller", y_traj_controller_);
    SmartDashboard.putData("Tuning/Swerve/Heading Traj Controller", heading_traj_controller_);
    SmartDashboard.putData("Tuning/Swerve/X Pose Controller", x_pose_controller_);
    SmartDashboard.putData("Tuning/Swerve/Y Pose Controller", y_pose_controller_);
    SmartDashboard.putData("Tuning/Swerve/Heading Pose Controller", heading_pose_controller_);

    // NT Publishers
    requested_state_pub_ =
        NetworkTableInstance.getDefault()
            .getStructArrayTopic("Swerve/Module States/Request", SwerveModuleState.struct)
            .publish();
    current_state_pub_ =
        NetworkTableInstance.getDefault()
            .getStructArrayTopic("Swerve/Module States/Current", SwerveModuleState.struct)
            .publish();
    requested_chassis_speeds_pub_ =
        NetworkTableInstance.getDefault()
            .getStructTopic("Swerve/Chassis Speeds/Request", ChassisSpeeds.struct)
            .publish();
    current_chassis_speeds_pub_ =
        NetworkTableInstance.getDefault()
            .getStructTopic("Swerve/Chassis Speeds/Current", ChassisSpeeds.struct)
            .publish();
    tractorbeam_pose_ =
        NetworkTableInstance.getDefault()
            .getStructTopic("Swerve/Tractor Beam/Pose", Pose2d.struct)
            .publish();

    SmartDashboard.putData(
        "Subsystems/Swerve/Overview",
        new Sendable() {
          @Override
          public void initSendable(SendableBuilder builder) {
            builder.setSmartDashboardType("SwerveDrive");

            builder.addDoubleProperty(
                "Front Left Angle", () -> io_.current_module_states_[0].angle.getRadians(), null);
            builder.addDoubleProperty(
                "Front Left Velocity",
                () -> io_.current_module_states_[0].speedMetersPerSecond,
                null);

            builder.addDoubleProperty(
                "Front Right Angle", () -> io_.current_module_states_[1].angle.getRadians(), null);
            builder.addDoubleProperty(
                "Front Right Velocity",
                () -> io_.current_module_states_[1].speedMetersPerSecond,
                null);

            builder.addDoubleProperty(
                "Back Left Angle", () -> io_.current_module_states_[2].angle.getRadians(), null);
            builder.addDoubleProperty(
                "Back Left Velocity",
                () -> io_.current_module_states_[2].speedMetersPerSecond,
                null);

            builder.addDoubleProperty(
                "Back Right Angle", () -> io_.current_module_states_[3].angle.getRadians(), null);
            builder.addDoubleProperty(
                "Back Right Velocity",
                () -> io_.current_module_states_[3].speedMetersPerSecond,
                null);

            builder.addDoubleProperty("Robot Angle", () -> io_.robot_yaw_.getRadians(), null);
          }
        });

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
    // Module States
    for (int i = 0; i < swerve_modules_.length; ++i) {
      io_.module_positions_[i] = swerve_modules_[i].getPosition(true);
      io_.current_module_states_[i] = swerve_modules_[i].getCurrentState();
      io_.requested_module_states_[i] = swerve_modules_[i].getTargetState();
    }
    // Control Inputs
    io_.joystick_left_x_ = OI.getDriverJoystickLeftX();
    io_.joystick_left_y_ = OI.getDriverJoystickLeftY();
    io_.joystick_right_x_ = OI.getDriverJoystickRightX();
    io_.joystick_pov = OI.getDriverJoystickPOV();

    // Position and Odom Info
    io_.robot_yaw_ =
        Rotation2d.fromRadians(MathUtil.angleModulus(pigeon_imu_.getYaw().getValue().in(Radians)));
    io_.current_chassis_speeds_ = kinematics_.toChassisSpeeds(io_.current_module_states_);
    io_.requested_chassis_speeds_ = kinematics_.toChassisSpeeds(io_.requested_module_states_);
    io_.current_pose_ = PoseEstimator.getInstance().getRobotPose();
  }

  @Override
  public synchronized void updateLogic(double timestamp) {
    io_.failing_to_reach_target =
        stallingDebouncer.calculate(
            !Util.epislonEquals(
                io_.current_chassis_speeds_,
                io_.requested_chassis_speeds_,
                Constants.DrivetrainConstants.FAILING_TO_REACH_TARGET_SPEEDS_TOLERANCE));
    if (OI.use_vision.getAsBoolean() == true) {
      request_parameters_.currentPose = io_.current_pose_;
    } else {
      request_parameters_.currentPose = new Pose2d(0, 0, io_.robot_yaw_);
    }
    switch (io_.drive_mode_) {
      case ROBOT_CENTRIC:
        {
          setControl(
              robot_centric_
                  // Drive forward with negative Y (forward)
                  .withVelocityX(-io_.joystick_left_y_ * io_.active_max_speed)
                  // Drive left with negative X (left)
                  .withVelocityY(-io_.joystick_left_x_ * io_.active_max_speed)
                  // Drive counterclockwise with negative X (left)
                  .withRotationalRate(
                      -io_.joystick_right_x_ * DrivetrainConstants.MAX_DRIVE_ANGULAR_RATE));
        }
        break;
      case FIELD_CENTRIC:
        {
          setControl(
              field_centric_
                  // Drive forward with negative Y (forward)
                  .withVelocityX(-io_.joystick_left_y_ * io_.active_max_speed)
                  // Drive left with negative X (left)
                  .withVelocityY(-io_.joystick_left_x_ * io_.active_max_speed)
                  // Drive counterclockwise with negative X (left)
                  .withRotationalRate(
                      -io_.joystick_right_x_ * DrivetrainConstants.MAX_DRIVE_ANGULAR_RATE));
        }
        break;
      case TARGET_FACING:
        {
          setControl(
              field_centric_target_facing_
                  // Drive forward with negative Y (forward)
                  .withVelocityX(
                      Util.clamp(
                          -io_.joystick_left_y_ * io_.active_max_speed,
                          DrivetrainConstants.MAX_TARGET_SPEED))
                  // Drive left with negative X (left)
                  .withVelocityY(
                      Util.clamp(
                          -io_.joystick_left_x_ * io_.active_max_speed,
                          DrivetrainConstants.MAX_TARGET_SPEED))
                  // Set Robots target rotation
                  .withTargetDirection(io_.target_rotation_));
        }
        break;
      case TRAJECTORY:
        {
          double x_velocity =
              io_.target_sample_.vx
                  + x_traj_controller_.calculate(io_.current_pose_.getX(), io_.target_sample_.x);
          double y_velocity =
              io_.target_sample_.vy
                  + y_traj_controller_.calculate(io_.current_pose_.getY(), io_.target_sample_.y);
          double omega =
              (io_.target_sample_.omega)
                  + heading_traj_controller_.calculate(
                      io_.current_pose_.getRotation().getRadians(), io_.target_sample_.heading);
          this.setControl(
              auto_request_
                  .withVelocityX(x_velocity)
                  .withVelocityY(y_velocity)
                  .withRotationalRate(omega));
        }
        break;
      case TRACTOR_BEAM:
        {
          io_.tractor_beam_scaling_factor_ =
              Math.sqrt(
                      Math.pow(io_.joystick_left_x_, 2)
                          + Math.pow(io_.joystick_left_y_, 2)
                          + Math.pow(io_.joystick_right_x_, 2))
                  * io_.active_max_speed;
          double x_velocity =
              // Util.clamp(
              x_pose_controller_.calculate(io_.current_pose_.getX(), io_.target_pose_.getX());
          // io_.tractor_beam_scaling_factor_);
          double y_velocity =
              // Util.clamp(
              y_pose_controller_.calculate(io_.current_pose_.getY(), io_.target_pose_.getY());
          // io_.tractor_beam_scaling_factor_);
          double omega =
              // Util.clamp(
              heading_pose_controller_.calculate(
                  io_.current_pose_.getRotation().getRadians(),
                  io_.target_pose_.getRotation().getRadians());
          // io_.tractor_beam_scaling_factor_);
          this.setControl(
              auto_request_
                  .withVelocityX(
                      Util.clamp(x_velocity, DrivetrainConstants.MAX_TRACTOR_BEAM_VELOCITY_SPEED))
                  .withVelocityY(
                      Util.clamp(y_velocity, DrivetrainConstants.MAX_TRACTOR_BEAM_VELOCITY_SPEED))
                  .withRotationalRate(
                      Util.clamp(omega, DrivetrainConstants.MAX_TRACTOR_BEAM_OMEGA_SPEED)));
        }
        break;
      case TIGHT_ROPE:
        {
          double ropeEndHandOffThreshold = 0.05;
          double x_velocity =
              x_pose_controller_.calculate(io_.current_pose_.getX(), io_.tight_rope_.poseA.getX());
          double y_velocity = 0;
          boolean pastA =
              io_.current_pose_.getY() > io_.tight_rope_.poseA.getY() + ropeEndHandOffThreshold;
          boolean pastB =
              io_.current_pose_.getY() < io_.tight_rope_.poseB.getY() - ropeEndHandOffThreshold;
          if (!pastA && !pastB) {
            y_velocity = -io_.joystick_left_x_ * io_.active_max_speed;
          } else if (pastA) {
            y_velocity =
                y_pose_controller_.calculate(
                    io_.current_pose_.getY(), io_.tight_rope_.poseA.getY());
          } else if (pastB) {
            y_velocity =
                y_pose_controller_.calculate(
                    io_.current_pose_.getY(), io_.tight_rope_.poseB.getY());
          }
          this.setControl(
              field_centric_target_facing_
                  .withTargetDirection(io_.tight_rope_.poseA.getRotation())
                  .withVelocityX(
                      Util.clamp(x_velocity, DrivetrainConstants.MAX_TRACTOR_BEAM_VELOCITY_SPEED))
                  .withVelocityY(
                      Util.clamp(y_velocity, DrivetrainConstants.MAX_TRACTOR_BEAM_VELOCITY_SPEED)));
        }
        break;
      case CRAWL:
        {
          if (io_.joystick_pov.isPresent()) {
            setControl(
                robot_centric_
                    // Drive forward with negative Y (forward)
                    .withVelocityX(
                        Math.round(io_.joystick_pov.get().getCos())
                            * DrivetrainConstants.CRAWL_DRIVE_SPEED)
                    // Drive left with negative X (left)
                    .withVelocityY(
                        Math.round(-io_.joystick_pov.get().getSin())
                            * DrivetrainConstants.CRAWL_DRIVE_SPEED)
                    .withRotationalRate(
                        -io_.joystick_right_x_ * DrivetrainConstants.CRAWL_DRIVE_SPEED));
          }
        }
        break;
      case IDLE:
        {
          setControl(new SwerveRequest.Idle());
        }
        break;
      default:
        break;
    }

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
    current_chassis_speeds_pub_.set(io_.current_chassis_speeds_);
    requested_chassis_speeds_pub_.set(io_.requested_chassis_speeds_);
    tractorbeam_pose_.set(io_.target_pose_);

    SmartDashboard.putString("Subsystems/Swerve/Mode", io_.drive_mode_.toString());
    SmartDashboard.putBoolean(
        "Subsystems/Swerve/FailingToReachTarget", io_.failing_to_reach_target);
    SmartDashboard.putNumber(
        "Subsystems/Swerve/Controller POV", io_.joystick_pov.orElse(new Rotation2d()).getDegrees());
    SmartDashboard.putString(
        "Subsystems/Swerve/Request Type", request_to_apply_.getClass().getSimpleName());
    SmartDashboard.putNumber(
        "Subsystems/Swerve/Driver Perspective", io_.drivers_station_perspective_.getDegrees());
    SmartDashboard.putNumber("Subsystems/Swerve/Yaw", io_.robot_yaw_.getDegrees());
    SmartDashboard.putNumber(
        "Subsystems/Swerve/FL Encoder",
        Units.rotationsToDegrees(swerve_modules_[0].getEncoderValue()));
    SmartDashboard.putNumber(
        "Subsystems/Swerve/FR Encoder",
        Units.rotationsToDegrees(swerve_modules_[1].getEncoderValue()));
    SmartDashboard.putNumber(
        "Subsystems/Swerve/BL Encoder",
        Units.rotationsToDegrees(swerve_modules_[2].getEncoderValue()));
    SmartDashboard.putNumber(
        "Subsystems/Swerve/BR Encoder",
        Units.rotationsToDegrees(swerve_modules_[3].getEncoderValue()));
    SmartDashboard.putNumber("Subsystems/Swerve/Active Max Speed", io_.active_max_speed);
    SmartDashboard.putBoolean(
        "Subsystems/Swerve/Failing To Reach Target", getFailingToReachTarget());
    SmartDashboard.putNumber("Subsystems/Swerve/Tractor Beam Error", getTractorBeamError());
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
   * updates the mode flag thats changes what request is applied to the drive train. If request
   * modes are ROBOT_CENTRIC or FIELD_CENTRIC the default request is updated.
   *
   * @param mode request drive mode
   */
  public synchronized void setDriveMode(DriveMode mode) {
    if (mode == DriveMode.FIELD_CENTRIC) io_.default_drive_mode_ = mode;
    if (mode == DriveMode.ROBOT_CENTRIC) io_.default_drive_mode_ = mode;
    io_.drive_mode_ = mode;
  }

  /** Updates the internal drive mode to the default teleop drive mode */
  public void restoreDefaultDriveMode() {
    io_.drive_mode_ = io_.default_drive_mode_;
  }

  /** Toggles between FIELD_CENTRIC and ROBOT_CENTRIC drive modes */
  public Command toggleFieldCentric() {
    return Commands.runOnce(
        () -> {
          if (io_.drive_mode_ == DriveMode.FIELD_CENTRIC) setDriveMode(DriveMode.ROBOT_CENTRIC);
          else setDriveMode(DriveMode.FIELD_CENTRIC);
        });
  }

  /**
   * Sets the drivers perspective for field relative requests
   *
   * @param perspective rotation to set as forward perspective
   */
  public void setDriverPerspective(Rotation2d perspective) {
    io_.drivers_station_perspective_ = perspective;
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
   * @return if robot at its target tractor Beam Pose
   */
  public boolean atTractorBeamPose() {
    return Util.epislonEquals(
        io_.current_pose_,
        io_.target_pose_,
        DrivetrainConstants.TRACTOR_BEAM_ROTATION_THRESHOLD,
        DrivetrainConstants.TRACTOR_BEAM_TARGET_DISTANCE);
  }

  /**
   * Sets the target rope points and rotation and begins TIGHT_ROPE mode !! Only locks to the line
   * along the Y axis !! pointA must have a higher Y than pointB pointA rotation is used to set the
   * robot rotation
   *
   * @param TightRope
   */
  public void setTightRope(TightRope tightrope) {
    io_.drive_mode_ = DriveMode.TIGHT_ROPE;
    io_.tight_rope_ = tightrope;
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
   * Allows the selection of max speed presets
   *
   * @param preset
   */
  public void setActiveSpeed(SpeedPresets preset) {
    io_.active_max_speed = DrivetrainConstants.MAX_DRIVE_SPEED * preset.speed_limit;
  }

  public boolean getFailingToReachTarget() {
    return io_.failing_to_reach_target;
  }

  public double getTractorBeamError() {
    double yError = Math.abs(y_pose_controller_.getError());
    double xError = Math.abs(x_pose_controller_.getError());
    return Math.sqrt(Math.pow(xError, 2) + Math.pow(yError, 2));
  }

  /**
   * Plain-Old-Data class holding the state of the swerve drivetrain. This encapsulates most data
   * that is relevant for telemetry or decision-making from the Swerve Drive.
   */
  public class SwerveDrivetrainPeriodicIo implements Logged {
    @Log.File public double active_max_speed = DrivetrainConstants.MAX_DRIVE_SPEED;
    @Log.File public SwerveModuleState[] current_module_states_, requested_module_states_;
    @Log.File public SwerveModulePosition[] module_positions_;
    @Log.File public DriveMode drive_mode_ = DriveMode.IDLE;
    @Log.File public DriveMode default_drive_mode_ = DriveMode.FIELD_CENTRIC;
    @Log.File public double joystick_left_x_ = 0.0;
    @Log.File public double joystick_left_y_ = 0.0;
    @Log.File public double joystick_right_x_ = 0.0;
    @Log.File public Optional<Rotation2d> joystick_pov = Optional.empty();
    @Log.File public Rotation2d drivers_station_perspective_ = new Rotation2d();
    @Log.File public Rotation2d robot_yaw_ = new Rotation2d();
    @Log.File public ChassisSpeeds current_chassis_speeds_ = new ChassisSpeeds();
    @Log.File public ChassisSpeeds requested_chassis_speeds_ = new ChassisSpeeds();
    @Log.File public Rotation2d target_rotation_ = new Rotation2d();
    @Log.File public double tractor_beam_scaling_factor_ = 0.0;
    @Log.File public Pose2d current_pose_ = new Pose2d();
    @Log.File public Pose2d target_pose_ = new Pose2d();
    @Log.File public boolean failing_to_reach_target = false;

    @Log.File
    public TightRope tight_rope_ = new TightRope(new Pose2d(), new Pose2d(), "Default Drivetrain");

    @Log.File
    public SwerveSample target_sample_ = new SwerveSample(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, null, null);
  }

  @Override
  public Logged getLoggingObject() {
    return io_;
  }
}
