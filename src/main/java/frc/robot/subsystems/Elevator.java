package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.FieldConstants;
import frc.mw_lib.ElevatorKinematics;
import frc.mw_lib.controls.TalonFXTuner;
import frc.mw_lib.subsystem.Subsystem;
import frc.mw_lib.util.Util;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.OI;
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
  private final MotionMagicVoltage elevator_request_;
  private final MotionMagicVoltage arm_request_;
  private BooleanSupplier elevator_at_minimum_;
  private Trigger reset_elevator_trigger_;
  private DigitalInput elevator_limit_switch_;
  private CANcoder arm_encoder_;

  private ElevatorKinematics kinematics_;

  private Mechanism2d system_mech_;
  private MechanismRoot2d mech_root_;
  private MechanismLigament2d elevator_mech_;
  private MechanismLigament2d elevator_max_mech_;
  private MechanismLigament2d arm_mech_;

  TalonFXTuner elevator_tuner_;
  TalonFXTuner arm_tuner_;

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

    // Hardware
    elevator_limit_switch_ = new DigitalInput(ElevatorConstants.ELEVATOR_LIMIT_SWITCH_PORT_NUMBER);
    elevator_master_ = new TalonFX(ElevatorConstants.ELEVATOR_MASTER_ID, "CANivore");
    elevator_follower_ = new TalonFX(ElevatorConstants.ELEVATOR_FOLLOWER_ID, "CANivore");
    arm_motor_ = new TalonFX(ElevatorConstants.ARM_MOTOR_ID, "CANivore");
    arm_encoder_ = new CANcoder(ElevatorConstants.ARM_ENCODER_ID);

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
        ElevatorConstants.ELEVATOR_MAX_HEIGHT;
    elevator_config_.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    elevator_config_.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        ElevatorConstants.ELEVATOR_MIN_HEIGHT;
    elevator_config_.CurrentLimits.StatorCurrentLimit =
        ElevatorConstants.ELEVATOR_STATOR_CURRENT_LIMIT;
    elevator_config_.CurrentLimits.StatorCurrentLimitEnable = true;

    elevator_config_.MotorOutput.Inverted = ElevatorConstants.ELEVATOR_MASTER_INVERSION_;
    elevator_master_.getConfigurator().apply(elevator_config_);
    elevator_config_.MotorOutput.Inverted = ElevatorConstants.ELEVATOR_FOLLOWER_INVERSION;
    elevator_follower_.getConfigurator().apply(elevator_config_);

    // Arm Configuration
    arm_config_ = new TalonFXConfiguration();
    // arm_config_.Feedback.FeedbackRemoteSensorID =
    // ElevatorConstants.ARM_ENCODER_ID;
    arm_config_.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    arm_config_.Feedback.SensorToMechanismRatio = ElevatorConstants.SENSOR_TO_MECHANISM_RATIO;
    arm_config_.Slot0 = ElevatorConstants.ARM_GAINS;
    arm_config_.MotionMagic.MotionMagicCruiseVelocity = ElevatorConstants.ARM_CRUISE_VELOCITY;
    arm_config_.MotionMagic.MotionMagicAcceleration = ElevatorConstants.ARM_ACCELERATION;
    arm_config_.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    arm_config_.ClosedLoopGeneral.ContinuousWrap = true;
    arm_motor_.getConfigurator().apply(arm_config_);

    // System Behavior Setup
    elevator_request_ = new MotionMagicVoltage(0);
    arm_request_ = new MotionMagicVoltage(0);

    elevator_at_minimum_ = () -> isElevatorAtMinimum();
    reset_elevator_trigger_ = new Trigger(elevator_at_minimum_);

    kinematics_ =
        new ElevatorKinematics(ElevatorConstants.MIN_ARM_LENGTH, ElevatorConstants.ARM_Z_OFFSET);

    reset_elevator_trigger_.onTrue(Commands.runOnce(() -> elevatorPoseReset()));

    // Mechanism Setup
    system_mech_ = new Mechanism2d(0, 0);
    mech_root_ = system_mech_.getRoot("Base", 0, 0);
    elevator_mech_ =
        mech_root_.append(
            new MechanismLigament2d(
                "Elevator",
                ElevatorConstants.ELEVATOR_MIN_HEIGHT,
                90,
                6,
                new Color8Bit(Color.kPurple)));
    arm_mech_ =
        elevator_mech_.append(
            new MechanismLigament2d(
                "Arm", ElevatorConstants.MIN_ARM_LENGTH, 0, 6, new Color8Bit(Color.kOrange)));
    elevator_max_mech_ =
        elevator_mech_.append(
            new MechanismLigament2d(
                "Elevator Max",
                ElevatorConstants.ELEVATOR_HEIGHT_ABOVE_PIVOT,
                0,
                6,
                new Color8Bit(Color.kPurple)));

    // System Tuning
    elevator_tuner_ =
        new TalonFXTuner(elevator_master_, new TalonFX[] {elevator_follower_}, "Elevator", this);
    // bindTuner(elevator_tuner_, 5, 10);

    arm_tuner_ = new TalonFXTuner(arm_motor_, "Arm", this);
    // bindTuner(arm_tuner_, 0.0, 0.5);
  }

  /** Called to reset and configure the subsystem */
  public void reset() {}

  /** Reads all sensors and stores periodic data */
  public void readPeriodicInputs(double timestamp) {
    io_.elevator_follower_rotations_ = elevator_follower_.getPosition().getValue().in(Rotations);
    io_.elevator_master_rotations_ = elevator_master_.getPosition().getValue().in(Rotations);
    io_.current_elevator_height =
        ((io_.elevator_master_rotations_ + io_.elevator_follower_rotations_) / 2)
            * ElevatorConstants.ELEVATOR_ROTATIONS_TO_METERS;
    // io_.current_arm_angle =
    // arm_encoder_.getAbsolutePosition().getValue().in(Radians);
    io_.current_arm_angle_ = arm_motor_.getPosition().getValue().in(Radians);
  }

  /** Computes updated outputs for the actuators */
  public void updateLogic(double timestamp) {
    switch (io_.current_target_config) {
      case L1:
        io_.target_arm_angle = Rotation2d.fromDegrees(-75).getRadians();
        io_.target_arm_height = FieldConstants.ReefHeight.L1.HEIGHT;
        break;
      case L2:
        io_.target_arm_angle = Rotation2d.fromDegrees(55).getRadians();
        io_.target_arm_height = FieldConstants.ReefHeight.L2.HEIGHT;
        break;
      case L3:
        io_.target_arm_angle = Rotation2d.fromDegrees(55).getRadians();
        io_.target_arm_height = FieldConstants.ReefHeight.L3.HEIGHT;
        break;
      case L4:
        io_.target_arm_angle = Rotation2d.fromDegrees(55).getRadians();
        io_.target_arm_height = FieldConstants.ReefHeight.L4.HEIGHT;
        break;
      case ALGAE_HIGH:
        io_.target_elevator_height = ElevatorConstants.ELEVATOR_MIN_HEIGHT;
        io_.target_arm_angle = 0;
        break;
      case ALGAE_LOW:
        io_.target_elevator_height = ElevatorConstants.ELEVATOR_MIN_HEIGHT;
        io_.target_arm_angle = 0;
        break;
      case SOURCE:
        io_.target_arm_angle = Units.degreesToRadians(230);
        io_.target_arm_height = FieldConstants.ReefHeight.L2.HEIGHT;
        break;
      case GROUND:
        io_.target_elevator_height = ElevatorConstants.ELEVATOR_MIN_HEIGHT;
        io_.target_arm_angle = 0;
        break;
      case PROCESSOR:
        io_.target_elevator_height = ElevatorConstants.ELEVATOR_MIN_HEIGHT;
        io_.target_arm_angle = 0;
        break;
      case BARGE:
        io_.target_elevator_height = ElevatorConstants.ELEVATOR_MIN_HEIGHT;
        io_.target_arm_angle = 0;
        break;
      case STOW:
        io_.target_elevator_height = ElevatorConstants.ELEVATOR_MIN_HEIGHT;
        io_.target_arm_angle = 0;
        break;
      case IDLE:
      default:
        io_.target_elevator_height = ElevatorConstants.ELEVATOR_MIN_HEIGHT;
        io_.target_arm_angle = 0;
        break;
    }

    io_.target_elevator_height =
        kinematics_.desiredElevatorZ(io_.target_arm_height, io_.target_arm_angle);
  }

  /** Writes the periodic outputs to actuators (motors and etc...) */
  public void writePeriodicOutputs(double timestamp) {
    if (OI.getDriverJoystickRightTriggerAxis() > 0.5) {
      elevator_master_.setControl(
          elevator_request_
              .withPosition(
                  io_.target_elevator_height / ElevatorConstants.ELEVATOR_ROTATIONS_TO_METERS)
              .withLimitReverseMotion(isElevatorAtMinimum()));
      elevator_follower_.setControl(new StrictFollower(elevator_master_.getDeviceID()));
      arm_motor_.setControl(
          arm_request_.withPosition(Units.radiansToRotations(io_.target_arm_angle)));
    } else {
      elevator_master_.setControl(new VoltageOut(0));
      elevator_follower_.setControl(new VoltageOut(0));
      arm_motor_.setControl(new VoltageOut(0));
    }
  }

  /** Outputs all logging information to the SmartDashboard */
  public void outputTelemetry(double timestamp) {
    double curr_arm_height =
        kinematics_.effectorZ(io_.current_elevator_height, io_.current_arm_angle_);
    SmartDashboard.putNumber("Current Arm Height", curr_arm_height);
    SmartDashboard.putNumber("Target Arm Height", io_.target_arm_height);

    SmartDashboard.putNumber("Arm Offset", kinematics_.calZOffset(io_.current_arm_angle_));
    SmartDashboard.putNumber("Target Elevator Height", io_.target_elevator_height);
    SmartDashboard.putNumber("Current Elevator Height", io_.current_elevator_height);
    SmartDashboard.putNumber("Current Arm Angle", io_.current_arm_angle_);
    SmartDashboard.putNumber("Target Arm Angle", io_.target_arm_angle);
    updateMechanism();
  }

  public void updateMechanism() {
    elevator_mech_.setLength(io_.current_elevator_height);
    arm_mech_.setAngle(Math.toDegrees(io_.current_arm_angle_) - 90);
    SmartDashboard.putData("Elevator System Mech", system_mech_);
  }

  /**
   * @return If the arm is within the threshold of its target
   */
  public boolean isArmAtTarget() {
    return Util.epislonEquals(
        io_.current_arm_angle_, io_.target_arm_angle, ElevatorConstants.ARM_TARGET_THRESHOLD);
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
    elevator_master_.setPosition(
        ElevatorConstants.ELEVATOR_MIN_HEIGHT / ElevatorConstants.ELEVATOR_ROTATIONS_TO_METERS);
    elevator_follower_.setPosition(
        ElevatorConstants.ELEVATOR_MIN_HEIGHT / ElevatorConstants.ELEVATOR_ROTATIONS_TO_METERS);
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
    @Log.File public TargetConfig current_target_config = TargetConfig.SOURCE;
    @Log.File public double current_elevator_height = 0;
    @Log.File public double target_elevator_height = 0;
    @Log.File public double current_arm_angle_ = 0;
    @Log.File public double target_arm_angle = 0;
    @Log.File public double target_arm_height = 0;
    @Log.File public double elevator_master_rotations_ = 0;
    @Log.File public double elevator_follower_rotations_ = 0;
    @Log.File public double elevator_average_position = 0;
  }

  /** Get logging object from subsystem */
  public Logged getLoggingObject() {
    return io_;
  }
}
