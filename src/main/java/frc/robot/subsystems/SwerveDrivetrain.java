/*
 * Copyright (C) Cross The Road Electronics.  All rights reserved.
 * License information can be found in CTRE_LICENSE.txt
 * For support and suggestions contact support@ctr-electronics.com or file
 * an issue tracker at https://github.com/CrossTheRoadElec/Phoenix-Releases
 */
package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.ProtobufPublisher;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.Util;
import frc.lib.subsystem.Subsystem;
import frc.lib.swerve.*;
import frc.lib.swerve.SwerveRequest.SwerveControlRequestParameters;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.subsystems.SwerveDrivetrain.SwerveDriverainPeriodicIo;
import frc.robot.Constants.DrivetrainConstants;

import monologue.Logged;
import monologue.Annotations.Log;

/**
 * Swerve Drive class utilizing CTR Electronics' Phoenix 6 API.
 * <p>
 * This class handles the kinematics, configuration, and odometry of a
 * swerve drive utilizing CTR Electronics devices. We recommend
 * that users use the Swerve Mechanism Generator in Tuner X to create
 * a template project that demonstrates how to use this class.
 * <p>
 * This class will construct the hardware devices internally, so the user
 * only specifies the constants (IDs, PID gains, gear ratios, etc).
 * Getters for these hardware devices are available.
 * <p>
 * If using the generator, the order in which modules are constructed is
 * Front Left, Front Right, Back Left, Back Right. This means if you need
 * the Back Left module, call {@code getModule(2);} to get the 3rd index
 * (0-indexed) module, corresponding to the Back Left module.
 */
public class SwerveDrivetrain extends Subsystem {
    private static SwerveDrivetrain instance;

    public static SwerveDrivetrain getInstance() {
        if (instance == null) {
            instance = new SwerveDrivetrain(DrivetrainConstants.FL_MODULE_CONSTANTS,
                    DrivetrainConstants.FR_MODULE_CONSTANTS, DrivetrainConstants.BL_MODULE_CONSTANTS,
                    DrivetrainConstants.BR_MODULE_CONSTANTS);
        }
        return instance;
    }

    // Drive Mode Selections
    public enum DriveMode {
        ROBOT_CENTRIC,
        FIELD_CENTRIC,
        TARGET,
        AUTONOMOUS,
        AUTONOMOUS_TARGET,
    }

    // Robot Hardware
    private final Pigeon2 pigeon_imu;
    private final SwerveModule[] swerve_modules;

    // Subsystem data class
    private SwerveDriverainPeriodicIo io_;

    // Drivetrain config
    final SwerveDriveKinematics kinematics;
    private final Translation2d[] module_locations;

    // Drive requests
    private SwerveRequest.ApplyChassisSpeeds auto_request;
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
    private StructArrayPublisher<SwerveModuleState> current_state_pub, requested_state_pub;
    private ProtobufPublisher<Pose2d> pp_pose_pub_;

    /**
     * Constructs a SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so user should not construct
     * the devices themselves. If they need the devices, they can access them
     * through getters in the classes.
     *
     * @param modules Constants for each specific module
     */
    public SwerveDrivetrain(SwerveModuleConstants... modules) {

        // make new io instance
        io_ = new SwerveDriverainPeriodicIo();
        io_.max_drive_speed_ = Constants.DrivetrainConstants.MAX_DRIVE_SPEED;

        // Setup the Pigeon IMU
        pigeon_imu = new Pigeon2(DrivetrainConstants.PIGEON2_ID, DrivetrainConstants.MODULE_CANBUS_NAME[0]);
        pigeon_imu.optimizeBusUtilization();

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
        field_centric = new SwerveRequest.FieldCentric()
                .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
                .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagic)
                .withDeadband(Constants.DrivetrainConstants.MAX_DRIVE_SPEED * 0.01)
                .withRotationalDeadband(Constants.DrivetrainConstants.MAX_DRIVE_ANGULAR_RATE * 0.01);
        robot_centric = new SwerveRequest.RobotCentric()
                .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
                .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagic)
                .withDeadband(Constants.DrivetrainConstants.MAX_DRIVE_SPEED * 0.01)
                .withRotationalDeadband(Constants.DrivetrainConstants.MAX_DRIVE_ANGULAR_RATE * 0.01);
        target_facing = new SwerveRequest.FieldCentricFacingAngle()
                .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
                .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagic)
                .withDeadband(Constants.DrivetrainConstants.MAX_DRIVE_SPEED * 0.01)
                .withRotationalDeadband(Constants.DrivetrainConstants.MAX_DRIVE_ANGULAR_RATE * 0.01);
        auto_request = new SwerveRequest.ApplyChassisSpeeds()
                .withDriveRequestType(SwerveModule.DriveRequestType.Velocity);
        request_parameters = new SwerveControlRequestParameters();
        request_to_apply = new SwerveRequest.Idle();

        // NT Publishers
        requested_state_pub = NetworkTableInstance.getDefault()
                .getStructArrayTopic("module_states/requested", SwerveModuleState.struct).publish();
        current_state_pub = NetworkTableInstance.getDefault()
                .getStructArrayTopic("module_states/current", SwerveModuleState.struct).publish();
        pp_pose_pub_ = NetworkTableInstance.getDefault()
                .getProtobufTopic("pp_target_pose", Pose2d.proto).publish();
    }

    @Override
    public void reset() {
        // 4 signals for each module + 2 for Pigeon2
        for (int i = 0; i < swerve_modules.length; ++i) {
            BaseStatusSignal.setUpdateFrequencyForAll(100, swerve_modules[i].getSignals());
            swerve_modules[i].optimizeCan();
            swerve_modules[i].resetToAbsolute();
        }
        BaseStatusSignal[] imuSignals = { pigeon_imu.getYaw() };
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

        io_.robot_yaw_ = Rotation2d.fromRadians(MathUtil.angleModulus(-pigeon_imu.getAngle() * Math.PI / 180));

        io_.chassis_speeds_ = kinematics.toChassisSpeeds(io_.current_module_states_);
        io_.field_relative_chassis_speed_ = ChassisSpeeds.fromRobotRelativeSpeeds(io_.chassis_speeds_, io_.robot_yaw_);
        io_.max_drive_speed_ = SmartDashboard.getNumber("Max Drive Speed", Constants.DrivetrainConstants.MAX_DRIVE_SPEED);
    }

    @Override
    public void updateLogic(double timestamp) {
        switch (io_.drive_mode_) {
            case ROBOT_CENTRIC:
                setControl(robot_centric
                        // Drive forward with negative Y (forward)
                        .withVelocityX(-io_.driver_joystick_leftY_ * io_.max_drive_speed_)
                        // Drive left with negative X (left)
                        .withVelocityY(-io_.driver_joystick_leftX_ * io_.max_drive_speed_)
                        // Drive counterclockwise with negative X (left)
                        .withRotationalRate(
                                -io_.driver_joystick_rightX_ * Constants.DrivetrainConstants.MAX_DRIVE_ANGULAR_RATE));
                break;
            case FIELD_CENTRIC:
                setControl(field_centric
                        // Drive forward with negative Y (forward)
                        .withVelocityX(-io_.driver_joystick_leftY_ * io_.max_drive_speed_)
                        // Drive left with negative X (left)
                        .withVelocityY(-io_.driver_joystick_leftX_ * io_.max_drive_speed_)
                        // Drive counterclockwise with negative X (left)
                        .withRotationalRate(
                                -io_.driver_joystick_rightX_ * Constants.DrivetrainConstants.MAX_DRIVE_ANGULAR_RATE));
                break;
            case TARGET:
                setControl(target_facing
                        // Drive forward with negative Y (forward)
                        .withVelocityX(
                                Util.clamp(-io_.driver_joystick_leftY_ * io_.max_drive_speed_,
                                        Constants.DrivetrainConstants.MAX_TARGET_SPEED))
                        // Drive left with negative X (left)
                        .withVelocityY(
                                Util.clamp(-io_.driver_joystick_leftX_ * io_.max_drive_speed_,
                                        Constants.DrivetrainConstants.MAX_TARGET_SPEED))
                        // Set Robots target rotation
                        .withTargetDirection(io_.target_rotation_)
                        // Use current robot rotation
                        .useGyroForRotation(io_.is_locked_with_gyro));
                break;
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

        io_.chassis_speed_magnitude_ = calculateChassisSpeedMagnitude(io_.chassis_speeds_);
    }

    @Override
    public void writePeriodicOutputs(double timestamp) {
        request_to_apply.apply(request_parameters, swerve_modules);
    }

    @Override
    public void outputTelemetry(double timestamp) {
        current_state_pub.set(io_.current_module_states_);
        requested_state_pub.set(io_.requested_module_states_);

        SmartDashboard.putNumber("Rotation Control/Target Rotation", io_.target_rotation_.getDegrees());
        SmartDashboard.putNumber("Rotation Control/Current Yaw", io_.robot_yaw_.getDegrees());
        SmartDashboard.putNumber("Debug/Driver Prespective", io_.drivers_station_perspective_.getDegrees());
        SmartDashboard.putNumber("Debug/Chassis Speed/X", io_.chassis_speeds_.vxMetersPerSecond);
        SmartDashboard.putNumber("Debug/Chassis Speed/Y", io_.chassis_speeds_.vyMetersPerSecond);
        SmartDashboard.putNumber("Debug/Chassis Speed/Omega", io_.chassis_speeds_.omegaRadiansPerSecond);
        SmartDashboard.putNumber("Debug/Chassis Speed/magnitude", io_.chassis_speed_magnitude_);

        // SmartDashboard.putData("X_Controler", DrivetrainConstants.X_CONTROLLER);
        // SmartDashboard.putData("Y_Controler", DrivetrainConstants.Y_CONTROLLER);
        // SmartDashboard.putData("T_Controler", DrivetrainConstants.T_CONTROLLER);
    }

    public Rotation2d getRobotRotation() {
        return io_.robot_yaw_;
    }

    /**
     * Configures the PathPlanner AutoBuilder
     */
    public void configurePathPlanner() {
        AutoBuilder.configureHolonomic(
                PoseEstimator.getInstance()::getFieldPose, // Supplier of current robot pose
                PoseEstimator.getInstance()::setRobotOdometry, // Consumer for seeding pose against auto
                this::getCurrentRobotChassisSpeeds,
                (speeds) -> this.setControl(auto_request.withSpeeds(speeds)), // Consumer of ChassisSpeeds
                getHolonomicFollowerConfig(),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this); // Subsystem for requirements
        PPHolonomicDriveController.setRotationTargetOverride(this::getAutoTargetRotation);

        // Logging callback for target robot pose
        PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
            // Do whatever you want with the pose here
            pp_pose_pub_.set(pose);
        });
    }

    public HolonomicPathFollowerConfig getHolonomicFollowerConfig() {
        double driveBaseRadius = 0;
        for (var moduleLocation : module_locations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }
        return new HolonomicPathFollowerConfig(new PIDConstants(5.0, 0.0, 0.001), // was 10
                new PIDConstants(7.3, 0, 0.07),
                5,
                driveBaseRadius,
                new ReplanningConfig(false, false),
                0.01);
    }

    public Command followPathCommand(String pathName) {
        return new FollowPathHolonomic(
                PathPlannerPath.fromPathFile(pathName),
                PoseEstimator.getInstance()::getFieldPose, // Supplier of current robot pose
                this::getCurrentRobotChassisSpeeds,
                (speeds) -> this.setControl(new SwerveRequest.ApplyChassisSpeeds().withSpeeds(speeds)), // Consumer of
                                                                                                        // ChassisSpeeds
                                                                                                        // to drive the
                                                                                                        // robot
                this.getHolonomicFollowerConfig(),

                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this); // Subsystem for requirements
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return Commands.run(() -> this.setControl(requestSupplier.get()));
    }

    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return io_.chassis_speeds_;
    }

    public ChassisSpeeds getFieldRelativeSpeeds(){
        return io_.field_relative_chassis_speed_;
    }

    /**
     * Takes the current orientation of the robot and makes it X forward for
     * field-relative maneuvers.
     */
    public void seedFieldRelative(Rotation2d offset) {
        pigeon_imu.setYaw(offset.getDegrees());
        io_.robot_yaw_ = Rotation2d.fromRadians(MathUtil.angleModulus(-pigeon_imu.getAngle() * Math.PI / 180));
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
     * <p>
     * This will zero the entire odometry, and place the robot at 0,0
     */
    public void tareEverything() {
        for (int i = 0; i < swerve_modules.length; ++i) {
            swerve_modules[i].resetPosition();
            swerve_modules[i].setWheelOffsets();
            io_.module_positions[i] = swerve_modules[i].getPosition(true);
        }
    }

    /**
     * Gets the raw value from the Robot IMU
     */
    public Rotation2d getImuYaw() {
        return io_.robot_yaw_;
    }

    /**
     * Returns the module locations in reference to the center of the robot as an
     * array
     * [FrontLeft, FrontRight, BackLeft, BackRight]
     */
    public SwerveModulePosition[] getModulePositions() {
        return io_.module_positions;
    }

    /**
     * Returns the module states of the swerve drive as an array
     * [FrontLeft, FrontRight, BackLeft, BackRight]
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
     * updates the mode flag thats changes what request is applied to the drive
     * train
     * 
     * @param mode drive to switch to [ROBOT_CENTRIC, FIELD_CENTRIC]
     */
    public void setDriveMode(DriveMode mode) {
        io_.drive_mode_ = mode;
    }

    public void toggleFieldCentric(){
        io_.drive_mode_ = (io_.drive_mode_ == DriveMode.FIELD_CENTRIC)?  DriveMode.ROBOT_CENTRIC : DriveMode.FIELD_CENTRIC;
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

    public double calculateChassisSpeedMagnitude(ChassisSpeeds chassis) {
        return Math.sqrt((chassis.vxMetersPerSecond * chassis.vxMetersPerSecond)
                + (chassis.vyMetersPerSecond * chassis.vyMetersPerSecond));
    }

    public void rotationTargetWithGyro(boolean state){
        io_.is_locked_with_gyro = state;
    }

    public void updateMaxDriveSpeed(double speed){
        io_.max_drive_speed_ = speed;
    }

    /**
     * Plain-Old-Data class holding the state of the swerve drivetrain.
     * This encapsulates most data that is relevant for telemetry or
     * decision-making from the Swerve Drive.
     */
    public class SwerveDriverainPeriodicIo implements Logged {
        @Log.File
        public SwerveModuleState[] current_module_states_, requested_module_states_;
        @Log.File
        public SwerveModulePosition[] module_positions;
        @Log.File
        public Rotation2d robot_yaw_ = new Rotation2d();
        @Log.File
        public double driver_joystick_leftX_ = 0.0;
        @Log.File
        public double driver_joystick_leftY_ = 0.0;
        @Log.File
        public double driver_joystick_rightX_ = 0.0;
        @Log.File
        public ChassisSpeeds chassis_speeds_ = new ChassisSpeeds();
        @Log.File
        public ChassisSpeeds field_relative_chassis_speed_ = new ChassisSpeeds();
        @Log.File
        public Rotation2d target_rotation_ = new Rotation2d();
        @Log.File
        public DriveMode drive_mode_ = DriveMode.FIELD_CENTRIC;
        @Log.File
        public double driver_POVx = 0.0;
        @Log.File
        public double driver_POVy = 0.0;
        @Log.File
        public Rotation2d drivers_station_perspective_ = new Rotation2d();
        @Log.File
        public double chassis_speed_magnitude_;
        @Log.File
        public boolean is_locked_with_gyro = false;
        @Log.File
        public double max_drive_speed_ = 0.0;
    }

    @Override
    public Logged getLoggingObject() {
        return io_;
    }
}