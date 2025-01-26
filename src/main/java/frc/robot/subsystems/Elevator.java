package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.mw_lib.subsystem.Subsystem;
import frc.mw_lib.util.Util;
import frc.robot.Constants.ElevatorConstants;
import java.util.function.BooleanSupplier;
import monologue.Annotations.Log;
import monologue.Logged;

public class Elevator extends Subsystem {
  // Singleton pattern
  private static Elevator elevatorInstance = null;

  public static Elevator getInstance() {
    if (elevatorInstance == null) {
      elevatorInstance = new Elevator();
    }
    return elevatorInstance;
  }

  // Elevator/Arm motors
  private TalonFX elevator_master_;
  private TalonFX elevator_follower_;
  private TalonFX arm_motor_;
  private TalonFXConfiguration elevator_config_;
  private TalonFXConfiguration arm_config_;
  private final MotionMagicExpoVoltage elevator_request_;
  private final MotionMagicExpoVoltage arm_request_;
  private BooleanSupplier elevator_at_minimum_;
  private Trigger reset_elevator_trigger_;
  private DigitalInput elevator_limit_switch_;
  private CANcoder arm_encoder_;

  // Enums for Elevator/Arm
  public enum TargetConfig {
    L1,
    L2,
    L3,
    L4,
    ALGAE_HIGH,
    ALGAE_LOW,
    SOURCE,
    GROUND,
    PROCESSOR,
    BARGE,
    STOW,
    IDLE
  }

  private ElevatorPeriodicIo io_;

  // Constructor
  public Elevator() {
    io_ = new ElevatorPeriodicIo();

    // Limit Switch: Elevator
    // Change channel once we find out what port it goes into on the RoboRIO
    elevator_limit_switch_ = new DigitalInput(ElevatorConstants.ELEVATOR_LIMIT_SWITCH_PORT_NUMBER);
    elevator_master_ = new TalonFX(ElevatorConstants.ELEVATOR_MASTER_ID);
    elevator_follower_ = new TalonFX(ElevatorConstants.ELEVATOR_FOLLOWER_ID);
    arm_motor_ = new TalonFX(ElevatorConstants.ARM_MOTOR_ID);
    arm_encoder_ = new CANcoder(ElevatorConstants.ARM_ENCODER_ID);
    elevator_config_ = new TalonFXConfiguration();

    elevator_config_.Feedback.SensorToMechanismRatio =
        ElevatorConstants.ELEVATOR_SENSOR_TO_MECHANISM_RATIO;
    elevator_config_.Slot0 = ElevatorConstants.ELEVATOR_GAINS;
    elevator_config_.MotionMagic.MotionMagicCruiseVelocity =
        ElevatorConstants.ELEVATOR_CRUISE_VELOCITY;
    elevator_config_.MotionMagic.MotionMagicAcceleration = ElevatorConstants.ELEVATOR_ACCELERATION;
    elevator_config_.MotionMagic.MotionMagicExpo_kV = ElevatorConstants.ELEVATOR_EXPO_KV;
    elevator_config_.MotionMagic.MotionMagicExpo_kA = ElevatorConstants.ELEVATOR_EXPO_KA;
    elevator_config_.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    elevator_config_.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    elevator_config_.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        ElevatorConstants.ELEVATOR_MAX_HEIGHT;

    elevator_config_.MotorOutput.Inverted = ElevatorConstants.ELEVATOR_MASTER_INVERSION_;
    elevator_master_.getConfigurator().apply(elevator_config_);
    elevator_config_.MotorOutput.Inverted = ElevatorConstants.ELEVATOR_FOLLOWER_INVERSION;
    elevator_follower_.getConfigurator().apply(elevator_config_);

    elevator_request_ = new MotionMagicExpoVoltage(0);
    arm_request_ = new MotionMagicExpoVoltage(0);

    elevator_at_minimum_ = () -> isElevatorAtMinimum();
    reset_elevator_trigger_ = new Trigger(elevator_at_minimum_);

    reset_elevator_trigger_.onTrue(Commands.runOnce(() -> elevatorPoseReset()));

    // Arm stuff
    arm_config_ = new TalonFXConfiguration();

    arm_config_.Feedback.RotorToSensorRatio = ElevatorConstants.ROTOR_TO_CENSOR_RATIO;
    arm_config_.Feedback.FeedbackRemoteSensorID = ElevatorConstants.ARM_ENCODER_ID;
    arm_config_.Feedback.SensorToMechanismRatio = ElevatorConstants.ARM_SENSOR_TO_MECHANISM_RATIO;
    arm_config_.Slot0 = ElevatorConstants.ARM_GAINS;
    arm_config_.MotionMagic.MotionMagicCruiseVelocity = ElevatorConstants.ARM_CRUISE_VELOCITY;
    arm_config_.MotionMagic.MotionMagicAcceleration = ElevatorConstants.ARM_ACCELERATION;
    arm_config_.MotionMagic.MotionMagicExpo_kV = ElevatorConstants.ARM_EXPO_KV;
    arm_config_.MotionMagic.MotionMagicExpo_kA = ElevatorConstants.ARM_EXPO_KA;
    arm_config_.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    arm_config_.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    arm_config_.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ElevatorConstants.ARM_LOWER_LIMIT;

    arm_motor_.getConfigurator().apply(arm_config_);
  }

  /** Called to reset and configure the subsystem */
  public void reset() {}

  /** Reads all sensors and stores periodic data */
  public void readPeriodicInputs(double timestamp) {
    io_.elevator_follower_position = elevator_follower_.getPosition().getValue().in(Rotations);
    io_.elevator_master_position = elevator_master_.getPosition().getValue().in(Rotations);
    io_.current_arm_angle = arm_encoder_.getAbsolutePosition().getValue().in(Radians);
  }

  /** Computes updated outputs for the actuators */
  public void updateLogic(double timestamp) {
    io_.elevator_average_position =
        (io_.elevator_master_position + io_.elevator_follower_position) / 2;

    switch (io_.current_target_config) {
      case L1:
        io_.target_elevator_height = 0;
        io_.target_arm_angle = 0;
        break;
      case L2:
        io_.target_elevator_height = 0;
        io_.target_arm_angle = 0;
        break;
      case L3:
        io_.target_elevator_height = 0;
        io_.target_arm_angle = 0;
        break;
      case L4:
        io_.target_elevator_height = 0;
        io_.target_arm_angle = 0;
        break;
      case ALGAE_HIGH:
        io_.target_elevator_height = 0;
        io_.target_arm_angle = 0;
        break;
      case ALGAE_LOW:
        io_.target_elevator_height = 0;
        io_.target_arm_angle = 0;
        break;
      case SOURCE:
        io_.target_elevator_height = 0;
        io_.target_arm_angle = 0;
        break;
      case GROUND:
        io_.target_elevator_height = 0;
        io_.target_arm_angle = 0;
        break;
      case PROCESSOR:
        io_.target_elevator_height = 0;
        io_.target_arm_angle = 0;
        break;
      case BARGE:
        io_.target_elevator_height = 0;
        io_.target_arm_angle = 0;
        break;
      case STOW:
        io_.target_elevator_height = 0;
        io_.target_arm_angle = 0;
        break;
      case IDLE:
      default:
        io_.target_elevator_height = 0;
        io_.target_arm_angle = 0;
        break;
    }
  }

  /** Writes the periodic outputs to actuators (motors and etc...) */
  public void writePeriodicOutputs(double timestamp) {
    elevator_master_.setControl(
        elevator_request_
            .withPosition(io_.target_elevator_height)
            .withLimitReverseMotion(isElevatorAtMinimum()));
    arm_motor_.setControl(arm_request_.withPosition(io_.target_arm_angle));
    elevator_follower_.setControl(new Follower(elevator_master_.getDeviceID(), true));
  }

  /** Outputs all logging information to the SmartDashboard */
  public void outputTelemetry(double timestamp) {}

  /**
   * @return If the arm is within the threshold of its target
   */
  public boolean isArmAtTarget() {
    return Util.epislonEquals(
        io_.current_arm_angle, io_.target_arm_angle, ElevatorConstants.ARM_TARGET_THRESHOLD);
  }

  /**
   * @return If the elevator is within the threshold of its target
   */
  public boolean isElevatorAtTarget() {
    return Util.epislonEquals(
        io_.current_elevator_height,
        io_.target_elevator_height,
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
    return io_.current_elevator_height <= ElevatorConstants.ELEVATOR_ZERO_THRESHOLD;
  }

  /** Reset the elevator pose to zero */
  public void elevatorPoseReset() {
    elevator_master_.setPosition(0);
    elevator_follower_.setPosition(0);
  }

  /** Reset the arm pose to home */
  public void armPoseReset() {
    arm_motor_.setPosition(ElevatorConstants.ARM_HOME_POSITION);
  }

  /** Reset the elevator position to zero and the arm to home */
  public void elevatorAndArmPoseReset() {
    elevatorPoseReset();
    armPoseReset();
  }

  /**
   * Set the target config of arm and elevator
   *
   * @param newConfig The new target config
   */
  public void setCurrentTargetConfig(TargetConfig newConfig) {
    io_.current_target_config = newConfig;
  }

  /**
   * @return If the elevator is within the threshold of zero and the limit switch is pressed
   */
  public boolean isElevatorAtMinimum() {
    return isLimitSwitchPressed() && isElevatorNearLimitSwitch();
  }

  public class ElevatorPeriodicIo implements Logged {
    // IO container for all variables
    @Log.File public TargetConfig current_target_config = TargetConfig.SOURCE;
    @Log.File public double current_elevator_height = 0;
    @Log.File public double target_elevator_height = 0;
    @Log.File public double current_arm_angle = 0;
    @Log.File public double target_arm_angle = 0;
    @Log.File public double elevator_master_position = 0;
    @Log.File public double elevator_follower_position = 0;
    @Log.File public double elevator_average_position = 0;
  }

  /** Get logging object from subsystem */
  public Logged getLoggingObject() {
    return io_;
  }
}
