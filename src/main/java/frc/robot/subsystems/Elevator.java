package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.ElevatorTargets.TargetType;
import frc.mw_lib.controls.TalonFXTuner;
import frc.mw_lib.subsystem.Subsystem;
import frc.mw_lib.util.MWPreferences;
import frc.mw_lib.util.NumUtil;
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

  // Mechanisms
  private StructArrayPublisher<Pose3d> current_stages_pub_;
  private StructPublisher<Pose3d> current_arm_pub_;

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
    CW,
    CCW
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
    elevator_config_.MotionMagic = ElevatorConstants.ELEVATOR_MAGIC_CONFIG;

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
    arm_config_.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    arm_config_.ClosedLoopGeneral.ContinuousWrap = false;
    arm_config_.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    arm_config_.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ArmConstants.ARM_FORWARD_LIMIT;
    arm_config_.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    arm_config_.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ArmConstants.ARM_REVERSE_LIMIT;
    arm_config_.MotionMagic = ArmConstants.ARM_MAGIC_CONFIG;
    arm_motor_.getConfigurator().apply(arm_config_);

    // Arm Encoder Config
    arm_encoder_config_ = new CANcoderConfiguration();
    arm_encoder_config_.MagnetSensor.SensorDirection = ArmConstants.ABSOLUTE_ENCODER_INVERSION;
    arm_encoder_config_.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
    arm_encoder_.getConfigurator().apply(arm_encoder_config_);

    // System Behavior Setup
    elevator_request_ = new MotionMagicVoltage(0);
    arm_request_ = new MotionMagicVoltage(0);

    // Mechanism Setup
    current_stages_pub_ =
        NetworkTableInstance.getDefault()
            .getStructArrayTopic("Components/Elevator Stages", Pose3d.struct)
            .publish();
    current_arm_pub_ =
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
        Commands.runOnce(() -> Elevator.getInstance().elevatorPositionReset())
            .ignoringDisable(true));
    SmartDashboard.putData(
        "Commands/Zero Arm",
        Commands.runOnce(() -> Elevator.getInstance().armPosReset()).ignoringDisable(true));
    arm_motor_.setPosition(
        readArmEncoder().getRotations()
            - (MWPreferences.getInstance().getPreferenceDouble("ArmEncoderOffset", 0)));
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

    // Default Behavior
    io_.target_elevator_height_ = io_.final_target_.getHeight();
    io_.target_arm_angle_ = io_.final_target_.getAngle().getRadians();

    // If Leaving collision Behavior
    if (io_.leaving_collision_target) {
      io_.target_arm_angle_ = TargetType.L4.getStagingAngle().get().getRadians();
      io_.target_elevator_height_ = io_.current_elevator_height_;
      if (isArmAtCurrentTarget()) {
        io_.target_elevator_height_ = io_.final_target_.getHeight();
      }
      if (isElevatorCloseToCurrentTarget() && isArmAtCurrentTarget()) {
        io_.target_arm_angle_ = io_.final_target_.getHeight();
        io_.leaving_collision_target = false;
      }
      // Behavior if Target has Staging Angle
    } else {
      // If Elevator is not at final target position set arm to staging angle
      if (!isElevatorCloseToTarget() && io_.final_target_.getStagingAngle().isPresent()) {
        io_.target_arm_angle_ = io_.final_target_.getStagingAngle().get().getRadians();
      }
    }

    // Elevator Safety
    if (io_.target_elevator_height_ < ElevatorConstants.ELEVATOR_HEIGHT_PIVOT_MIN) {
      DataLogManager.log(
          "ERROR: Target Elevator Height: "
              + io_.target_elevator_height_
              + " Min Elevator Height: "
              + ElevatorConstants.ELEVATOR_HEIGHT_PIVOT_MIN);
      io_.target_elevator_height_ = ElevatorConstants.ELEVATOR_HEIGHT_PIVOT_MIN;
    }
    if (io_.target_elevator_height_ > ElevatorConstants.ELEVATOR_HEIGHT_PIVOT_MAX) {
      // DataLogManager.log(
      // "ERROR: Target Elevator Height: "
      // + io_.target_elevator_height_
      // + " Max Elevator Height: "
      // + ElevatorConstants.ELEVATOR_HEIGHT_PIVOT_MAX);
      // io_.target_elevator_height_ = ElevatorConstants.ELEVATOR_HEIGHT_PIVOT_MAX;
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
        "Subsystems/Arm/Target Angle (Degrees)", Units.radiansToDegrees(io_.target_arm_angle_));
    SmartDashboard.putString("Subsystems/Elevator/Target", io_.final_target_.toString());
    SmartDashboard.putBoolean("Subsystems/Elevator/At Target", isElevatorAtTarget());
    SmartDashboard.putBoolean("Subsystems/Arm/At Target", isArmAtTarget());
    updateMechanism(
        current_stages_pub_,
        current_arm_pub_,
        io_.current_elevator_height_,
        io_.current_arm_angle_);
  }

  public boolean targetIsL1() {
    return io_.final_target_ == TargetType.L1;
  }

  /**
   * Publishes a JointSpaceSolution of the Elevator to NT
   *
   * @param stages_pub publisher for elevator stages
   * @param arm_pub publisher for arm
   */
  public void updateMechanism(
      StructArrayPublisher<Pose3d> stages_pub,
      StructPublisher<Pose3d> arm_pub,
      double elevator_height,
      double arm_angle) {
    stages_pub.set(
        new Pose3d[] {
          new Pose3d(
              0,
              0,
              (elevator_height - ElevatorConstants.ELEVATOR_HEIGHT_PIVOT_MIN) / 2
                  + ElevatorConstants.ELEVATOR_HEIGHT_PIVOT_MIN,
              Rotation3d.kZero),
          new Pose3d(0, 0, elevator_height, Rotation3d.kZero)
        });
    arm_pub.set(new Pose3d(0, 0, elevator_height, new Rotation3d(0, arm_angle, 0)));
  }

  public void updateMechanism(
      StructArrayPublisher<Pose3d> stages_pub, StructPublisher<Pose3d> arm_pub, TargetType target) {
    updateMechanism(stages_pub, arm_pub, target.getHeight(), target.getAngle().getRadians());
  }

  /**
   * Reads the arm encoder and handles wrapping
   *
   * @return raw arm encoder value without wrapping
   */
  private Rotation2d readArmEncoder() {
    double value = arm_encoder_.getAbsolutePosition().getValue().in(Degrees);
    if (Math.abs(value) > 180) return Rotation2d.fromDegrees(value + (-Math.copySign(360, value)));
    return Rotation2d.fromDegrees(value);
  }

  /**
   * @return If the arm is within the threshold of its final target
   */
  public boolean isArmAtTarget() {
    return NumUtil.epislonEquals(
        io_.current_arm_angle_,
        io_.final_target_.getAngle().getRadians(),
        ArmConstants.ARM_TARGET_THRESHOLD);
  }

  /**
   * @return If the elevator is within the threshold of its final target
   */
  public boolean isElevatorAtTarget() {
    return NumUtil.epislonEquals(
        io_.current_elevator_height_,
        io_.final_target_.getHeight(),
        ElevatorConstants.ELEVATOR_TARGET_THRESHOLD);
  }

  /**
   * @return If the elevator is within the threshold of its final target
   */
  public boolean isElevatorCloseToTarget() {
    return NumUtil.epislonEquals(
        io_.current_elevator_height_,
        io_.final_target_.getHeight(),
        ElevatorConstants.ELEVATOR_TARGET_THRESHOLD_CLOSE);
  }

  /**
   * @return If both the arm and elevator are at there targets
   */
  public boolean isElevatorAndArmAtTarget() {
    return isElevatorAtTarget() && isArmAtTarget();
  }

  public boolean isArmAtCurrentTarget() {
    return NumUtil.epislonEquals(
        io_.current_arm_angle_, io_.target_arm_angle_, ArmConstants.ARM_TARGET_THRESHOLD);
  }

  public boolean isElevatorAtCurrentTarget() {
    return NumUtil.epislonEquals(
        io_.current_elevator_height_,
        io_.target_elevator_height_,
        ElevatorConstants.ELEVATOR_TARGET_THRESHOLD);
  }

  public boolean isElevatorCloseToCurrentTarget() {
    return NumUtil.epislonEquals(
        io_.current_elevator_height_,
        io_.target_elevator_height_,
        ElevatorConstants.ELEVATOR_TARGET_THRESHOLD_CLOSE);
  }

  /**
   * @return If both the arm and elevator are at there targets
   */
  public boolean isElevatorAndArmAtCurrentTarget() {
    return isElevatorAtCurrentTarget() && isArmAtCurrentTarget();
  }

  public boolean isArmHung() {
    return io_.current_arm_angle_ > Units.degreesToRadians(-60);
  }

  public double getCurrentAngle() {
    return io_.current_arm_angle_;
  }

  public boolean isElevatorHung() {
    return io_.current_elevator_height_ > 1.51;
  }

  public boolean isElevatorAndArmHung() {
    return isArmHung() && isElevatorHung();
  }

  /** Reset the elevator pos to zero */
  public void elevatorPositionReset() {
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

  /**
   * Adjust the stored target setpoint by an inch in offset_type direction
   *
   * @param offset_type direction to move the translation point
   */
  public void setOffset(OffsetType offset_type) {
    switch (offset_type) {
      case UP:
        io_.final_target_.offsetHeight(Units.inchesToMeters(1));
        break;
      case DOWN:
        io_.final_target_.offsetHeight(Units.inchesToMeters(-1));
        break;
      case CW:
        io_.final_target_.offsetAngle(Rotation2d.fromDegrees(0.5));
        break;
      case CCW:
      default:
        io_.final_target_.offsetAngle(Rotation2d.fromDegrees(-0.5));
        break;
    }
  }

  /** Removes any applied offsets to the currently selected target */
  public void resetManualOffsets() {
    io_.final_target_.resetOffsets();
    io_.final_target_.resetOffsets();
  }

  /**
   * Sets the target for arm and elevator
   *
   * @param target
   */
  public synchronized void setTarget(TargetType new_target) {
    if (new_target == io_.final_target_) {
      return;
    }
    if (io_.final_target_.getCollision() && new_target != TargetType.L4_SAFETY) {
      io_.leaving_collision_target = true;
    } else {
      io_.leaving_collision_target = false;
    }
    io_.final_target_ = new_target;
  }

  /**
   * Gets the current target of the system
   *
   * @return current target
   */
  public TargetType getTarget() {
    return io_.final_target_;
  }

  /**
   * Binds supplied tuner to controller triggers
   *
   * @param tuner tuner to enable
   * @param pos1 first test position for MotionMagicVoltage
   * @param pos2 second test position for MotionMagicVoltage
   */
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

  public class ElevatorPeriodicIo implements Logged {
    // IO container for all variables
    @Log.File public double current_elevator_height_ = 0;
    @Log.File public double current_arm_angle_ = 0;
    @Log.File public double target_elevator_height_ = ElevatorConstants.ELEVATOR_HEIGHT_PIVOT_MIN;
    @Log.File public double target_arm_angle_ = Units.degreesToRadians(-90);
    @Log.File public double elevator_master_rotations_ = 0;
    @Log.File public double elevator_follower_rotations_ = 0;
    @Log.File public TargetType final_target_ = TargetType.CORAL_INTAKE;
    @Log.File public boolean leaving_collision_target = false;
  }

  /** Get logging object from subsystem */
  public Logged getLoggingObject() {
    return io_;
  }
}
