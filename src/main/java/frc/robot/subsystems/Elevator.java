package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Fahrenheit;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.ElevatorKinematics;
import frc.lib.ElevatorKinematics.JointSpaceTarget;
import frc.lib.ElevatorPlanner;
import frc.lib.ElevatorTargets.TargetType;
import frc.lib.FieldRegions;
import frc.mw_lib.controls.TalonFXTuner;
import frc.mw_lib.subsystem.Subsystem;
import frc.mw_lib.util.MWPreferences;
import frc.mw_lib.util.Util;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.OI;
import frc.robot.commands.SetDefaultStow;
import monologue.Annotations.Log;
import monologue.Logged;

public class Elevator extends Subsystem {
  // Singleton pattern
  private static Elevator elevator_instance_ = null;

  public static Elevator getInstance() {
    if (elevator_instance_ == null) {
      elevator_instance_ = new Elevator();
    }
    return elevator_instance_;
  }

  // Hardware
  private TalonFX elevator_master_;
  private TalonFX elevator_follower_;
  private TalonFX arm_motor_;
  private TalonFXConfiguration elevator_config_;
  private TalonFXConfiguration arm_config_;
  private MotionMagicVoltage elevator_request_;
  private MotionMagicVoltage arm_request_;
  private CANcoder arm_encoder_;
  private CANcoderConfiguration arm_encoder_config_;

  // Control Behavior
  private ElevatorKinematics kinematics_;
  private ElevatorPlanner planner_;

  // Mechanisms
  private StructArrayPublisher<Pose3d> stages_pub_;
  private StructPublisher<Pose3d> arm_pub_;

  TalonFXTuner elevator_tuner_;
  TalonFXTuner arm_tuner_;

  // Enums for Elevator/Arm
  public enum SpeedLimit {
    CORAL,
    ALGAE,
    SAFETY,
    L4
  }

  public enum OffsetType {
    UP,
    DOWN,
    LEFT,
    RIGHT
  }

  private ElevatorPeriodicIo io_;

  // Constructor
  public Elevator() {
    io_ = new ElevatorPeriodicIo();

    // Hardware
    elevator_master_ = new TalonFX(ElevatorConstants.ELEVATOR_MASTER_ID);
    elevator_follower_ = new TalonFX(ElevatorConstants.ELEVATOR_FOLLOWER_ID);
    arm_motor_ = new TalonFX(ArmConstants.ARM_MOTOR_ID);
    arm_encoder_ = new CANcoder(ArmConstants.ARM_ENCODER_ID);

    // Elevator Config
    elevator_config_ = new TalonFXConfiguration();
    elevator_config_.Slot0 = ElevatorConstants.ELEVATOR_GAINS;
    elevator_config_.MotionMagic.MotionMagicCruiseVelocity =
        ElevatorConstants.ELEVATOR_CRUISE_VELOCITY;
    elevator_config_.MotionMagic.MotionMagicAcceleration = ElevatorConstants.ELEVATOR_ACCEL;
    elevator_config_.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    elevator_config_.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    elevator_config_.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        ElevatorConstants.ELEVATOR_HEIGHT_PIVOT_MAX;
    elevator_config_.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    elevator_config_.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        ElevatorConstants.ELEVATOR_HEIGHT_PIVOT_MIN;
    elevator_config_.CurrentLimits.StatorCurrentLimit =
        ElevatorConstants.ELEVATOR_STATOR_CURRENT_LIMIT;
    elevator_config_.CurrentLimits.StatorCurrentLimitEnable = true;

    elevator_config_.MotorOutput.Inverted = ElevatorConstants.ELEVATOR_MASTER_INVERSION;
    elevator_master_.getConfigurator().apply(elevator_config_);
    elevator_config_.MotorOutput.Inverted = ElevatorConstants.ELEVATOR_FOLLOWER_INVERSION;
    elevator_follower_.getConfigurator().apply(elevator_config_);

    // Arm Configuration
    arm_config_ = new TalonFXConfiguration();
    arm_config_.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    arm_config_.Feedback.SensorToMechanismRatio = ArmConstants.SENSOR_TO_MECHANISM_RATIO;
    arm_config_.MotorOutput.Inverted = ArmConstants.ARM_FOLLOWER_INVERSION;
    arm_config_.Slot0 = ArmConstants.ARM_GAINS;
    arm_config_.MotionMagic.MotionMagicCruiseVelocity = ArmConstants.CORAL_ARM_CRUISE_VELOCITY;
    arm_config_.MotionMagic.MotionMagicAcceleration = ArmConstants.CORAL_ARM_ACCELERATION;
    arm_config_.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    arm_config_.ClosedLoopGeneral.ContinuousWrap = false;
    arm_config_.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    arm_config_.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ArmConstants.ARM_FORWARD_LIMIT;
    arm_config_.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    arm_config_.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ArmConstants.ARM_REVERSE_LIMIT;
    arm_motor_.getConfigurator().apply(arm_config_);

    // Arm Encoder Config
    arm_encoder_config_ = new CANcoderConfiguration();
    arm_encoder_config_.MagnetSensor.SensorDirection = ArmConstants.ABSOLUTE_ENCODER_INVERSION;
    arm_encoder_config_.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
    arm_encoder_.getConfigurator().apply(arm_encoder_config_);

    // System Behavior Setup
    elevator_request_ = new MotionMagicVoltage(0);
    arm_request_ = new MotionMagicVoltage(0);

    kinematics_ = new ElevatorKinematics(ArmConstants.ARM_LENGTH, ArmConstants.ARM_WIDTH);
    planner_ =
        new ElevatorPlanner(
            kinematics_,
            Constants.ElevatorConstants.SUBDIVISION_PER_METER,
            Constants.ElevatorConstants.SUBDIVISION_FOLLOW_DIST);

    // Mechanism Setup
    stages_pub_ =
        NetworkTableInstance.getDefault()
            .getStructArrayTopic("Components/Elevator/Stages", Pose3d.struct)
            .publish();
    arm_pub_ =
        NetworkTableInstance.getDefault().getStructTopic("Components/Arm", Pose3d.struct).publish();

    // System Tuning
    elevator_tuner_ =
        new TalonFXTuner(elevator_master_, new TalonFX[] {elevator_follower_}, "Elevator", this);
    // bindTuner(elevator_tuner_, 5, 10);

    arm_tuner_ = new TalonFXTuner(arm_motor_, "Arm", this);
    // bindTuner(arm_tuner_, 0.0, 0.5);

    // SmartDashboard.putData("Set Arm Encoder Offset", Commands.runOnce(() -> ));
    SmartDashboard.putData(
        "Commands/Reset Arm & Elevator Manual Offsets",
        Commands.runOnce(() -> resetManualOffsets()).ignoringDisable(true));
    // Sync Elevator and Arm Sensor to "Home" Position
    SmartDashboard.putData(
        "Commands/Zero Elevator",
        Commands.runOnce(() -> Elevator.getInstance().elevatorPosReset()).ignoringDisable(true));
    SmartDashboard.putData(
        "Commands/Zero Arm",
        Commands.runOnce(() -> Elevator.getInstance().armPosReset()).ignoringDisable(true));
    arm_motor_.setPosition(
        readArmEncoder().getRotations()
            - (MWPreferences.getInstance().getPreferenceDouble("ArmEncoderOffset", 0)));

    readPeriodicInputs(0);
    buildPlan(io_.final_target_); // update final_target_to change default
  }

  /** Called to reset and configure the subsystem */
  public void reset() {
    setDefaultCommand(new SetDefaultStow());
  }

  /** Reads all sensors and stores periodic data */
  public void readPeriodicInputs(double timestamp) {
    io_.elevator_follower_rotations_ = elevator_follower_.getPosition().getValue().in(Rotations);
    io_.elevator_master_rotations_ = elevator_master_.getPosition().getValue().in(Rotations);
    io_.current_elevator_height_ =
        io_.elevator_master_rotations_ * ElevatorConstants.ELEVATOR_ROTATIONS_TO_METERS
            + ElevatorConstants.ELEVATOR_HEIGHT_PIVOT_MIN;
    io_.current_arm_angle_ = (arm_motor_.getPosition().getValue().in(Radians));
  }

  /** Computes updated outputs for the actuators */
  public void updateLogic(double timestamp) {
    io_.currentTranslation =
        kinematics_.jointSpaceToTranslation(io_.current_elevator_height_, io_.current_arm_angle_);
    if (systemAtTarget(io_.current_target) && planner_.hasPath()) {
      io_.current_target = planner_.nextTarget(io_.currentTranslation);
    }

    io_.target_elevator_height_ = io_.current_target.pivot_height;
    io_.target_arm_angle_ = io_.current_target.pivot_angle;

    // elevator safteys
    if (io_.target_elevator_height_ < ElevatorConstants.ELEVATOR_HEIGHT_PIVOT_MIN) {
      // DataLogManager.log(
      // "ERROR: Target Elevator Height: "
      // + io_.target_elevator_height_
      // + " Min Elevator Height: "
      // + ElevatorConstants.ELEVATOR_HEIGHT_PIVOT_MIN);
      io_.target_elevator_height_ = ElevatorConstants.ELEVATOR_HEIGHT_PIVOT_MIN;
    }
    if (io_.target_elevator_height_ > ElevatorConstants.ELEVATOR_HEIGHT_PIVOT_MAX) {
      // DataLogManager.log(
      // "ERROR: Target Elevator Height: "
      // + io_.target_elevator_height_
      // + " Min Elevator Height: "
      // + ElevatorConstants.ELEVATOR_HEIGHT_PIVOT_MAX);
      io_.target_elevator_height_ = ElevatorConstants.ELEVATOR_HEIGHT_PIVOT_MAX;
    }
  }

  /** Writes the periodic outputs to actuators (motors and etc...) */
  public void writePeriodicOutputs(double timestamp) {
    elevator_master_.setControl(
        elevator_request_.withPosition(
            (io_.target_elevator_height_ - ElevatorConstants.ELEVATOR_HEIGHT_PIVOT_MIN)
                / ElevatorConstants.ELEVATOR_ROTATIONS_TO_METERS));
    elevator_follower_.setControl(new StrictFollower(elevator_master_.getDeviceID()));
    arm_motor_.setControl(
        arm_request_.withPosition(Units.radiansToRotations(io_.target_arm_angle_)));
  }

  /** Outputs all logging information to the SmartDashboard */
  public void outputTelemetry(double timestamp) {
    SmartDashboard.putNumber(
        "Subsystems/Elevator/Target Height (Meters)", io_.target_elevator_height_);
    SmartDashboard.putNumber(
        "Subsystems/Elevator/Current Height (Meters)", io_.current_elevator_height_);
    SmartDashboard.putNumber(
        "Subsystems/Elevator/Distance from Zero (Inches)",
        Units.metersToInches(
            io_.current_elevator_height_ - ElevatorConstants.ELEVATOR_HEIGHT_PIVOT_MIN));
    SmartDashboard.putNumber(
        "Subsystems/Elevator/Motor Temp Master (Fahrenheit)",
        elevator_master_.getDeviceTemp().getValue().in(Fahrenheit));
    SmartDashboard.putNumber(
        "Subsystems/Elevator/Motor Temp Follower (Fahrenheit)",
        elevator_follower_.getDeviceTemp().getValue().in(Fahrenheit));
    SmartDashboard.putNumber(
        "Subsystems/Arm/Current Angle (Degrees)", Units.radiansToDegrees(io_.current_arm_angle_));
    SmartDashboard.putNumber(
        "Subsystems/Arm/Target Angle (Degrees)",
        Units.radiansToDegrees(io_.current_target.pivot_angle));
    SmartDashboard.putNumber(
        "Subsystems/Arm/Current Height (Meters)", io_.currentTranslation.getZ());
    SmartDashboard.putNumber("Subsystems/Arm/Current X (Meters)", io_.currentTranslation.getX());
    SmartDashboard.putNumber(
        "Subsystems/Arm/Absolute Encoder (Rotations)", readArmEncoder().getRotations());
    SmartDashboard.putString("Subsystems/Elevator/Target", io_.final_target_.toString());
    SmartDashboard.putString("Subsystems/Elevator/Pending Target", io_.current_target.toString());
    SmartDashboard.putBoolean("Subsystems/Elevator/At Target", isElevatorAtTarget());
    SmartDashboard.putBoolean("Subsystems/Arm/At Target", isArmAtTarget());

    SmartDashboard.putNumber(
        "Subsystems/Arm/KinematicsTesting Angle Deg",
        Units.radiansToDegrees(
            kinematics_.translationToJointSpace(io_.currentTranslation).pivot_angle));
    SmartDashboard.putNumber(
        "Subsystems/Arm/KinematicsTesting Pivot Height",
        kinematics_.translationToJointSpace(io_.currentTranslation).pivot_height);

    updateMechanism();
  }

  private void updateMechanism() {
    stages_pub_.set(
        new Pose3d[] {
          new Pose3d(),
          new Pose3d(
              0,
              0,
              (io_.current_elevator_height_ - ElevatorConstants.ELEVATOR_HEIGHT_PIVOT_MIN) / 2
                  + ElevatorConstants.ELEVATOR_HEIGHT_PIVOT_MIN,
              new Rotation3d()),
          new Pose3d(0, 0, io_.current_elevator_height_, new Rotation3d())
        });
    arm_pub_.set(
        new Pose3d(
            0,
            0,
            io_.current_elevator_height_ + 0.012,
            new Rotation3d(0, io_.current_arm_angle_, 0)));
  }

  private Rotation2d readArmEncoder() {
    return Rotation2d.fromRotations(arm_encoder_.getAbsolutePosition().getValueAsDouble());
  }

  private boolean armAtTarget(JointSpaceTarget target) {
    return Util.epislonEquals(
        io_.current_arm_angle_, target.pivot_angle, ArmConstants.ARM_TARGET_THRESHOLD);
  }

  private boolean elevatorAtTarget(JointSpaceTarget target) {
    return Util.epislonEquals(
        io_.current_elevator_height_,
        target.pivot_height,
        ElevatorConstants.ELEVATOR_TARGET_THRESHOLD);
  }

  private boolean systemAtTarget(JointSpaceTarget target) {
    return elevatorAtTarget(target) && armAtTarget(target);
  }

  /**
   * @return If the arm is within the threshold of its final target
   */
  public boolean isArmAtTarget() {
    return armAtTarget(
        kinematics_.translationToJointSpace(io_.final_target_.getTarget().translation));
  }

  /**
   * @return If the elevator is within the threshold of its final target
   */
  public boolean isElevatorAtTarget() {
    return elevatorAtTarget(
        kinematics_.translationToJointSpace(io_.final_target_.getTarget().translation));
  }

  /**
   * @return If both the arm and elevator are at there targets
   */
  public boolean isElevatorAndArmAtTarget() {
    return isElevatorAtTarget() && isArmAtTarget();
  }

  /**
   * @return If the arm is possibly over the reef
   */
  public boolean isArmInDangerZone() {
    return io_.current_arm_angle_ > Constants.ArmConstants.DANGER_ARM_ANGLE;
  }

  /**
   * @return If the elevator is within threshold of the zero pose
   */
  public boolean isElevatorNearLimitSwitch() {
    return io_.current_elevator_height_ <= ElevatorConstants.ELEVATOR_ZERO_THRESHOLD;
  }

  /** Reset the elevator pos to zero */
  public void elevatorPosReset() {
    elevator_master_.setPosition(0);
    elevator_follower_.setPosition(0);
  }

  /** Sync Arm position to Arm encoder */
  public void armPosReset() {
    MWPreferences.getInstance().setPreference("ArmEncoderOffset", readArmEncoder().getRotations());
    arm_motor_.setPosition(
        readArmEncoder().getRotations()
            - (MWPreferences.getInstance().getPreferenceDouble("ArmEncoderOffset", 0)));
  }

  public void setOffset(OffsetType offset_type) {
    switch (offset_type) {
      case UP:
        io_.final_target_.offsetY(0.0254);
        break;
      case DOWN:
        io_.final_target_.offsetY(-0.0254);
        break;
      case LEFT:
        io_.final_target_.offsetX(-0.0254);
        break;
      case RIGHT:
      default:
        io_.final_target_.offsetX(0.0254);
        break;
    }
  }

  /**
   * Sets the target for arm and elevator
   *
   * @param target
   */
  public void setTarget(TargetType new_target) {
    if (new_target == io_.final_target_) {
      return;
    }

    buildPlan(new_target);
  }

  public void buildPlan(TargetType new_target) {
    TargetType old_target = io_.final_target_;
    io_.final_target_ = new_target;

    // TODO: reciving new targets when not at a known target
    Translation3d[] waypoints =
        new Translation3d[1 + old_target.getExitTrj().length + new_target.getEnterTrj().length + 1];
    waypoints[0] =
        kinematics_.jointSpaceToTranslation(io_.current_elevator_height_, io_.current_arm_angle_);
    int index = 1;
    for (Translation3d t : old_target.getExitTrj()) {
      waypoints[index] = t;
      index++;
    }
    for (Translation3d t : new_target.getEnterTrj()) {
      waypoints[index] = t;
      index++;
    }
    waypoints[waypoints.length - 1] = new_target.getTarget().translation;

    planner_.plan(waypoints);

    if (planner_.hasPath()) {
      io_.current_target =
          planner_.nextTarget(
              kinematics_.jointSpaceToTranslation(
                  io_.current_elevator_height_, io_.current_arm_angle_));
    }
  }

  public TargetType getTarget() {
    return io_.final_target_;
  }

  public void resetManualOffsets() {
    io_.final_target_.resetXOffset();
    io_.final_target_.resetYOffset();
  }

  public void stowElevator() {
    setTarget(TargetType.CORAL_INTAKE);
  }

  public void setSpeedLimit(SpeedLimit limit) {
    if (limit == io_.current_speed_limit) {
      return;
    }

    if (limit == SpeedLimit.CORAL) {
      arm_config_.MotionMagic.MotionMagicCruiseVelocity = ArmConstants.CORAL_ARM_CRUISE_VELOCITY;
      arm_config_.MotionMagic.MotionMagicAcceleration = ArmConstants.CORAL_ARM_ACCELERATION;
    } else if (limit == SpeedLimit.ALGAE) {
      arm_config_.MotionMagic.MotionMagicCruiseVelocity = ArmConstants.ALGAE_ARM_CRUISE_VELOCITY;
      arm_config_.MotionMagic.MotionMagicAcceleration = ArmConstants.ALGAE_ARM_ACCELERATION;
    } else if (limit == SpeedLimit.SAFETY) {
      arm_config_.MotionMagic.MotionMagicCruiseVelocity = ArmConstants.SAFETY_ARM_CRUISE_VELOCITY;
      arm_config_.MotionMagic.MotionMagicAcceleration = ArmConstants.SAFETY_ARM_ACCELERATION;
    } else if (limit == SpeedLimit.L4) {
      arm_config_.MotionMagic.MotionMagicCruiseVelocity = ArmConstants.CORAL_ARM_CRUISE_VELOCITY;
      arm_config_.MotionMagic.MotionMagicAcceleration = ArmConstants.L4_ARM_ACCEL;
    }
    arm_motor_.getConfigurator().apply(arm_config_);
    io_.current_speed_limit = limit;
  }

  public void bindTuner(TalonFXTuner tuner, double pos1, double pos2) {
    tuner.bindSetpoint(new MotionMagicVoltage(pos1), OI.getDriverJoystickAButtonTrigger());
    tuner.bindSetpoint(new MotionMagicVoltage(pos2), OI.getDriverJoystickYButtonTrigger());
    tuner.bindSetpoint(new VoltageOut(-1), OI.getDriverJoystickXButtonTrigger());
    tuner.bindSetpoint(new VoltageOut(1), OI.getDriverJoystickBButtonTrigger());
    tuner.bindDynamicForward(OI.getOperatorJoystickAButtonTrigger());
    tuner.bindDynamicReverse(OI.getOperatorJoystickBButtonTrigger());
    tuner.bindQuasistaticForward(OI.getOperatorJoystickXButtonTrigger());
    tuner.bindQuasistaticReverse(OI.getOperatorJoystickYButtonTrigger());
  }

  public boolean canExtendForBarge() {
    return FieldRegions.ALGAE_REGIONS[0].contains(PoseEstimator.getInstance().getRobotPose());
  }

  public SpeedLimit getCurrSpeedLimit() {
    return io_.current_speed_limit;
  }

  public class ElevatorPeriodicIo implements Logged {
    // IO container for all variables
    @Log.File public double current_elevator_height_ = 0;
    @Log.File public double target_elevator_height_ = ElevatorConstants.ELEVATOR_HEIGHT_PIVOT_MIN;
    @Log.File public double current_arm_angle_ = 0;
    @Log.File public double target_arm_angle_ = Units.degreesToRadians(-90);
    @Log.File public double elevator_master_rotations_ = 0;
    @Log.File public double elevator_follower_rotations_ = 0;
    @Log.File public Translation3d currentTranslation = new Translation3d();
    @Log.File public SpeedLimit current_speed_limit = SpeedLimit.CORAL;
    // the final goal
    @Log.File public TargetType final_target_ = TargetType.TEST;

    // all targets between the curent pose and the final target
    @Log.File
    public JointSpaceTarget current_target =
        new JointSpaceTarget(target_elevator_height_, target_arm_angle_);
  }

  /** Get logging object from subsystem */
  public Logged getLoggingObject() {
    return io_;
  }
}
