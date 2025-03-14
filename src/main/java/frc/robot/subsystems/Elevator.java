package frc.robot.subsystems;

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
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.ElevatorKinematics;
import frc.lib.ElevatorTargets.Target;
import frc.lib.FieldRegions;
import frc.lib.TargetData;
import frc.lib.TargetData.ControlType;
import frc.mw_lib.controls.TalonFXTuner;
import frc.mw_lib.subsystem.Subsystem;
import frc.mw_lib.util.MWPreferences;
import frc.mw_lib.util.Util;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.OI;
import frc.robot.commands.SetDefaultStow;
import java.util.function.BooleanSupplier;
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
  private BooleanSupplier elevator_at_minimum_;
  private Trigger reset_elevator_trigger_;
  private DigitalInput elevator_limit_switch_;
  private CANcoder arm_encoder_;
  private CANcoderConfiguration arm_encoder_config_;

  // Control Behavior
  private ElevatorKinematics kinematics_;

  // Mechanisms
  private StructArrayPublisher<Pose3d> stages_pub_;
  private StructPublisher<Pose3d> arm_pub_;

  TalonFXTuner elevator_tuner_;
  TalonFXTuner arm_tuner_;

  // Enums for Elevator/Arm
  public enum ControlMode {
    END_EFFECTOR,
    PIVOT
  }

  public enum SpeedLimit {
    CORAL,
    ALGAE
  }

  public enum OffsetType {
    ELEVATOR_UP,
    ELEVATOR_DOWN,
    ARM_CCW,
    ARM_CW
  }

  private ElevatorPeriodicIo io_;

  // Constructor
  public Elevator() {
    io_ = new ElevatorPeriodicIo();

    // Hardware
    elevator_limit_switch_ = new DigitalInput(ElevatorConstants.ELEVATOR_LIMIT_SWITCH_PORT_NUMBER);
    elevator_master_ = new TalonFX(ElevatorConstants.ELEVATOR_MASTER_ID, "CANivore");
    elevator_follower_ = new TalonFX(ElevatorConstants.ELEVATOR_FOLLOWER_ID, "CANivore");
    arm_motor_ = new TalonFX(ArmConstants.ARM_MOTOR_ID, "CANivore");
    arm_encoder_ = new CANcoder(ArmConstants.ARM_ENCODER_ID, "CANivore");

    // Elevator Config
    elevator_config_ = new TalonFXConfiguration();
    elevator_config_.Slot0 = ElevatorConstants.ELEVATOR_GAINS;
    elevator_config_.MotionMagic.MotionMagicCruiseVelocity =
        ElevatorConstants.ELEVATOR_CRUISE_VELOCITY;
    elevator_config_.MotionMagic.MotionMagicAcceleration = ElevatorConstants.ELEVATOR_ACCEL;
    elevator_config_.MotionMagic.MotionMagicExpo_kA = ElevatorConstants.ELEVATOR_EXPO_KA;
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
    arm_config_.Slot0 = ArmConstants.ARM_GAINS;
    arm_config_.MotionMagic.MotionMagicCruiseVelocity = ArmConstants.CORAL_ARM_CRUISE_VELOCITY;
    arm_config_.MotionMagic.MotionMagicAcceleration = ArmConstants.CORAL_ARM_ACCELERATION;
    arm_config_.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    arm_config_.ClosedLoopGeneral.ContinuousWrap = false;
    arm_config_.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    arm_config_.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ArmConstants.ARM_FORWARD_LIMT;
    arm_config_.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    arm_config_.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ArmConstants.ARM_REVERSE_LIMT;
    arm_motor_.getConfigurator().apply(arm_config_);

    // Arm Encoder Config
    arm_encoder_config_ = new CANcoderConfiguration();
    arm_encoder_config_.MagnetSensor.SensorDirection =
        SensorDirectionValue.CounterClockwise_Positive;
    arm_encoder_config_.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
    arm_encoder_.getConfigurator().apply(arm_encoder_config_);

    // System Behavior Setup
    elevator_request_ = new MotionMagicVoltage(0);
    arm_request_ = new MotionMagicVoltage(0);

    elevator_at_minimum_ = () -> isElevatorAtMinimum();
    reset_elevator_trigger_ = new Trigger(elevator_at_minimum_);

    kinematics_ = new ElevatorKinematics(ArmConstants.ARM_LENGTH, ArmConstants.ARM_WIDTH);

    reset_elevator_trigger_.onTrue(Commands.runOnce(() -> elevatorPosReset()));

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
        "Commands/Zero Elevator & Arm",
        Commands.runOnce(() -> Elevator.getInstance().elevatorAndArmPosReset())
            .ignoringDisable(true));
    arm_motor_.setPosition(
        arm_encoder_.getAbsolutePosition().getValueAsDouble()
            - (MWPreferences.getInstance().getPreferenceDouble("ArmEncoderOffset", 0)));
    elevatorPosReset();
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
        (((io_.elevator_master_rotations_ + io_.elevator_follower_rotations_) / 2)
                * ElevatorConstants.ELEVATOR_ROTATIONS_TO_METERS)
            + ElevatorConstants.ELEVATOR_HEIGHT_PIVOT_MIN;
    io_.current_arm_angle_ = arm_motor_.getPosition().getValue().in(Radians);
  }

  /** Computes updated outputs for the actuators */
  public void updateLogic(double timestamp) {
    if (io_.target_.getStagingArmAngle().isPresent()) {
      if (isElevatorAtTarget()) {
        io_.target_arm_angle_ = io_.target_.getAngle();
      } else {
        io_.target_arm_angle_ = io_.target_.getStagingArmAngle().get();
      }
    } else {
      io_.target_arm_angle_ = io_.target_.getAngle();
    }
    switch (io_.current_control_mode_) {
      case END_EFFECTOR:
        io_.target_elevator_height_ =
            kinematics_.desiredElevatorZ(io_.target_.getHeight(), io_.target_arm_angle_);
        break;
      case PIVOT:
        io_.target_elevator_height_ = io_.target_.getHeight();
        break;
    }

    if (io_.target_elevator_height_ < ElevatorConstants.ELEVATOR_HEIGHT_PIVOT_MIN) {
      DataLogManager.log(
          "ERROR: Target Elevator Height: "
              + io_.target_elevator_height_
              + " Min Elevator Height: "
              + ElevatorConstants.ELEVATOR_HEIGHT_PIVOT_MIN);
      io_.target_elevator_height_ = ElevatorConstants.ELEVATOR_HEIGHT_PIVOT_MIN;
    }
    if (io_.target_elevator_height_ > ElevatorConstants.ELEVATOR_HEIGHT_PIVOT_MAX) {
      DataLogManager.log(
          "ERROR: Target Elevator Height: "
              + io_.target_elevator_height_
              + " Min Elevator Height: "
              + ElevatorConstants.ELEVATOR_HEIGHT_PIVOT_MAX);
      io_.target_elevator_height_ = ElevatorConstants.ELEVATOR_HEIGHT_PIVOT_MAX;
    }
  }

  /** Writes the periodic outputs to actuators (motors and etc...) */
  public void writePeriodicOutputs(double timestamp) {
    elevator_master_.setControl(
        elevator_request_
            .withPosition(
                (io_.target_elevator_height_ - ElevatorConstants.ELEVATOR_HEIGHT_PIVOT_MIN)
                    / ElevatorConstants.ELEVATOR_ROTATIONS_TO_METERS)
            .withLimitReverseMotion(isElevatorAtMinimum()));
    elevator_follower_.setControl(new StrictFollower(elevator_master_.getDeviceID()));
    arm_motor_.setControl(arm_request_.withPosition(io_.target_arm_angle_.getRotations()));
  }

  /** Outputs all logging information to the SmartDashboard */
  public void outputTelemetry(double timestamp) {
    SmartDashboard.putString(
        "Subsystems/Elevator/Control Mode", io_.current_control_mode_.toString());
    SmartDashboard.putNumber("Subsystems/Elevator/Target Height", io_.target_elevator_height_);
    SmartDashboard.putNumber("Subsystems/Elevator/Current Height", io_.current_elevator_height_);
    SmartDashboard.putNumber("Subsystems/Elevator/Manual Offset", io_.target_.getHeightOffset());
    SmartDashboard.putString(
        "Subsystems/Elevator/Motor Temp Master", elevator_master_.getDeviceTemp().toString());
    SmartDashboard.putString(
        "Subsystems/Elevator/Motor Temp Follower", elevator_follower_.getDeviceTemp().toString());

    SmartDashboard.putString("Subsystems/Arm/Control Mode", io_.current_control_mode_.toString());
    SmartDashboard.putNumber("Subsystems/Arm/Current Angle", io_.current_arm_angle_);
    SmartDashboard.putNumber("Subsystems/Arm/Target Angle", io_.target_arm_angle_.getRadians());
    SmartDashboard.putNumber(
        "Subsystems/Arm/Current Height",
        kinematics_.effectorZ(io_.current_elevator_height_, io_.current_arm_angle_));
    SmartDashboard.putNumber(
        "Subsystems/Arm/Height Offset", kinematics_.calZOffset(io_.current_arm_angle_));
    SmartDashboard.putNumber(
        "Subsystems/Arm/Absolute Encoder",
        arm_encoder_.getAbsolutePosition().getValue().in(Rotations));
    SmartDashboard.putNumber(
        "Subsystems/Arm/Manual Offset", io_.target_.getAngleOffset().getRadians());
    updateMechanism();
  }

  public void updateMechanism() {
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

  /**
   * @return If the arm is within the threshold of its target
   */
  public boolean isArmAtTarget() {
    if(io_.target_.getStagingArmAngle().isPresent() && isArmAtStagingAngle()) {
      return false;
    }
    return Util.epislonEquals(
        io_.current_arm_angle_,
        io_.target_arm_angle_.getRadians(),
        ArmConstants.ARM_TARGET_THRESHOLD);
  }

  /**
   * @return If the arm is within the threshold of its target
   */
  public boolean isArmAtStagingAngle() {
    if (io_.target_.getStagingArmAngle().isPresent()) {
      return Util.epislonEquals(
          io_.current_arm_angle_,
          io_.target_.getStagingArmAngle().get().getRadians(),
          ArmConstants.ARM_TARGET_THRESHOLD);
    }
    return false;
  }

  /**
   * @return If the elevator is within the threshold of its target
   */
  public boolean isElevatorAtTarget() {
    return Util.epislonEquals(
        io_.current_elevator_height_,
        io_.target_elevator_height_,
        ElevatorConstants.ELEVATOR_TARGET_THRESHOLD);
  }

  /**
   * @return If both the arm and elevator are at there targets
   */
  public boolean isElevatorAndArmAtTarget() {
    return isElevatorAtTarget() && isArmAtTarget();
  }

  /**
   * @return If the limit switch is pressed
   */
  public boolean isLimitSwitchPressed() {
    return !elevator_limit_switch_.get();
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
    MWPreferences.getInstance()
        .setPreference("ArmEncoderOffset", arm_encoder_.getAbsolutePosition().getValueAsDouble());
    arm_motor_.setPosition(
        arm_encoder_.getAbsolutePosition().getValueAsDouble()
            - (MWPreferences.getInstance().getPreferenceDouble("ArmEncoderOffset", 0)));
  }

  /** Reset the elevator position to zero and the arm to home */
  public void elevatorAndArmPosReset() {
    elevatorPosReset();
    armPosReset();
  }

  public void setOffset(OffsetType offset_type) {
    switch (offset_type) {
      case ELEVATOR_UP:
        io_.target_.offsetHeight(0.0254);
        break;
      case ELEVATOR_DOWN:
        io_.target_.offsetHeight(-0.0254);
        break;
      case ARM_CCW:
        io_.target_.offsetAngle(Rotation2d.fromDegrees(-1));
        break;
      case ARM_CW:
      default:
        io_.target_.offsetAngle(Rotation2d.fromDegrees(1));
        break;
    }
  }

  public void setTarget(Target target) {
    io_.target_ = target;
    if (target.getControlType() == ControlType.PIVOT) {
      io_.current_control_mode_ = ControlMode.PIVOT;
    } else if (target.getControlType() == ControlType.EFFECTOR) {
      io_.current_control_mode_ = ControlMode.END_EFFECTOR;
    }
  }

  public Target getTarget() {
    return io_.target_;
  }

  public void resetManualOffsets() {
    io_.target_.resetAngleOffset();
    io_.target_.resetHeightOffset();
  }

  public void stowElevator() {
    setTarget(Target.STOW);
  }

  public void setSpeedLimit(SpeedLimit limit) {
    if (limit == SpeedLimit.CORAL) {
      arm_config_.MotionMagic.MotionMagicCruiseVelocity = ArmConstants.CORAL_ARM_CRUISE_VELOCITY;
      arm_config_.MotionMagic.MotionMagicAcceleration = ArmConstants.CORAL_ARM_ACCELERATION;
    } else if (limit == SpeedLimit.ALGAE) {
      arm_config_.MotionMagic.MotionMagicCruiseVelocity = ArmConstants.ALGAE_ARM_CRUISE_VELOCITY;
      arm_config_.MotionMagic.MotionMagicAcceleration = ArmConstants.ALGAE_ARM_ACCELERATION;
    }
    arm_motor_.getConfigurator().apply(arm_config_);
  }

  /**
   * @return If the elevator is within the threshold of zero and the limit switch is pressed
   */
  public boolean isElevatorAtMinimum() {
    return isLimitSwitchPressed() && isElevatorNearLimitSwitch();
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

  public class ElevatorPeriodicIo implements Logged {
    // IO container for all variables
    @Log.File public ControlMode current_control_mode_ = ControlMode.PIVOT;
    @Log.File public double current_elevator_height_ = 0;
    @Log.File public double target_elevator_height_ = ElevatorConstants.ELEVATOR_HEIGHT_PIVOT_MIN;
    @Log.File public double current_arm_angle_ = 0;
    @Log.File public Target target_ = Target.STOW;
    @Log.File public Rotation2d target_arm_angle_ = Rotation2d.fromDegrees(-90);
    @Log.File public double elevator_master_rotations_ = 0;
    @Log.File public double elevator_follower_rotations_ = 0;
    @Log.File public TargetData target_data_ = target_.getLoggingObject();
  }

  /** Get logging object from subsystem */
  public Logged getLoggingObject() {
    return io_;
  }
}
