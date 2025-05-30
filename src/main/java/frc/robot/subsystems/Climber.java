// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.thethriftybot.ThriftyNova;
import com.thethriftybot.ThriftyNova.CurrentType;
import com.thethriftybot.ThriftyNova.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.mw_lib.logging.Elastic;
import frc.mw_lib.subsystem.RemovableSubsystem;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.Pickup.PickupMode;
import monologue.Annotations.Log;
import monologue.Logged;

public class Climber extends RemovableSubsystem {

  private TalonFX strap_motor_;
  private ControlRequest strap_request_;
  private PositionVoltage strap_position_request_;
  private VoltageOut strap_voltage_request_;
  private TalonFXConfiguration strap_config_;

  private Encoder prong_counter_;
  private PIDController prong_controller_;
  private Spark prong_motor_;
  private ThriftyNova arm_motor_;

  public enum ClimberMode {
    DISABLED,
    PRECLIMB,
    STAGING,
    PRESET,
    DEPLOYING,
    DEPLOYED,
    RETRACTED
  }

  // Singleton pattern
  private static Climber climber_instance_ = null;

  public static Climber getInstance() {
    if (climber_instance_ == null) {
      climber_instance_ = new Climber();
    }
    return climber_instance_;
  }

  /** Class Members */
  private ClimberPeriodicIo io_;

  private Climber() {
    // Create io object first in subsystem configuration
    io_ = new ClimberPeriodicIo();

    if (isEnabled()) {
      prong_motor_ = new Spark(ClimberConstants.PRONG_ID);
      prong_motor_.setInverted(true);

      arm_motor_ = new ThriftyNova(ClimberConstants.ARM_ID);
      arm_motor_.setInverted(false);
      arm_motor_.setMaxCurrent(CurrentType.STATOR, ClimberConstants.ARM_SUPPLY_CURRENT_LIMIT);
      arm_motor_.setMotorType(MotorType.NEO);

      strap_motor_ = new TalonFX(ClimberConstants.STRAP_ID, "CANivore");
      strap_motor_.getConfigurator().apply(ClimberConstants.STRAP_GAINS);

      strap_position_request_ = new PositionVoltage(0.0);
      strap_position_request_.withSlot(0);
      strap_voltage_request_ = new VoltageOut(0);

      strap_config_ = new TalonFXConfiguration();
      strap_config_.MotorOutput.Inverted = ClimberConstants.STRAP_INVERSION;
      strap_config_.MotorOutput.NeutralMode = NeutralModeValue.Brake;

      prong_counter_ =
          new Encoder(
              ClimberConstants.PRONG_ID_A, ClimberConstants.PRONG_ID_B, false, EncodingType.k2X);
      prong_controller_ = new PIDController(ClimberConstants.PRONG_P, 0, ClimberConstants.PRONG_D);

      reset();
    }
  }

  /**
   * This function should be logic and code to fully reset your subsystem. This is called during
   * initialization, and should handle I/O configuration and initializing data members.
   */
  @Override
  public void reset() {
    prong_counter_.reset();
  }

  /**
   * Inside this function, all of the SENSORS should be read into variables stored in the PeriodicIO
   * class defined below. There should be no calls to output to actuators, or any logic within this
   * function.
   */
  @Override
  public void readPeriodicInputs(double timestamp) {
    io_.current_prong_rotations_ = prong_counter_.get();
    io_.strap_motor_current_ = strap_motor_.getPosition().getValueAsDouble();
  }

  /**
   * Inside this function, all of the LOGIC should compute updates to output variables in the
   * PeriodicIO class defined below. There should be no calls to read from sensors or write to
   * actuators in this function.
   */
  @Override
  public void updateLogic(double timestamp) {
    strap_request_ = strap_voltage_request_;
    switch (io_.current_mode_) {
      case PRECLIMB:
        io_.deploying_start_time_ = 0;
        break;
      case STAGING:
        io_.target_prong_rotations_ = ClimberConstants.PRONG_PRESET_COUNT;
        prong_controller_.setSetpoint(io_.target_prong_rotations_);
        double vcomp = 11.0 / RobotController.getBatteryVoltage(); // Tuned at 11.0v
        io_.prong_motor_demand = prong_controller_.calculate(io_.current_prong_rotations_) * vcomp;

        if (io_.current_prong_rotations_ >= ClimberConstants.PRONG_PRESET_COUNT) {
          io_.current_mode_ = ClimberMode.PRESET;
        }
        break;
      case PRESET:
        io_.prong_motor_demand = ClimberConstants.PRONG_HOLD_SPEED;
        // wait
        break;
      case DEPLOYING:
        io_.prong_motor_demand = ClimberConstants.PRONG_DEPLOY_SPEED;
        io_.target_arm_output_ = ClimberConstants.ARM_DEPLOY_SPEED;
        if (io_.deploying_start_time_ == 0) {
          io_.deploying_start_time_ = timestamp;
        }
        if (timestamp - io_.deploying_start_time_ >= ClimberConstants.DEPLOYING_TIME) {
          io_.current_mode_ = ClimberMode.DEPLOYED;
        }
        break;
      case DEPLOYED:
        io_.target_arm_output_ = ClimberConstants.ARM_HOLD_SPEED;
        io_.prong_motor_demand = ClimberConstants.PRONG_DEPLOY_SPEED / 2;
        // Wait for transition
        break;
      case RETRACTED:
        io_.target_arm_output_ = 0;
        io_.prong_motor_demand = 0;
        strap_request_ =
            strap_position_request_.withPosition(
                io_.target_strap_rotations_ + io_.strap_motor_target_offset);
        break;
      case DISABLED:
      default:
        prong_counter_.reset();
        break;
    }
  }

  /**
   * Inside this function actuator OUTPUTS should be updated from data contained in the PeriodicIO
   * class defined below. There should be little to no logic contained within this function, and no
   * sensors should be read.
   */
  @Override
  public void writePeriodicOutputs(double timestamp) {
    strap_motor_.setControl(strap_request_);
    prong_motor_.set(io_.prong_motor_demand);
    arm_motor_.set(io_.target_arm_output_);
  }

  /**
   * Inside this function telemetry should be output to smartdashboard. The data should be collected
   * out of the PeriodicIO class instance defined below. There should be no sensor information read
   * in this function nor any outputs made to actuators within this function. Only publish to
   * smartdashboard here.
   */
  @Override
  public void outputTelemetry(double timestamp) {
    SmartDashboard.putNumber(
        "Subsystems/Climber/Strap Target (Rotations)", io_.target_strap_rotations_);
    SmartDashboard.putNumber(
        "Subsystems/Climber/Prong Target (Rotations)", io_.target_prong_rotations_);
    SmartDashboard.putNumber(
        "Subsystems/Climber/Prong Current (Rotations)", io_.current_prong_rotations_);
    SmartDashboard.putNumber("Subsystems/Climber/Arm Target (Duty Cycle)", io_.target_arm_output_);
    SmartDashboard.putString("Subsystems/Climber/Mode", io_.current_mode_.toString());
  }

  public void nextStage() {
    switch (io_.current_mode_) {
      case DISABLED:
        io_.current_mode_ = ClimberMode.PRECLIMB;
        // The elevators default cmd will set to climb
        Elastic.selectTab("Climb");
        break;
      case PRECLIMB:
        if (Elevator.getInstance().isElevatorAndArmAtTarget()) {
          io_.current_mode_ = ClimberMode.STAGING;
        }
        break;
      case PRESET:
        io_.current_mode_ = ClimberMode.DEPLOYING;
        break;
      case DEPLOYED:
        io_.current_mode_ = ClimberMode.RETRACTED;
        io_.target_strap_rotations_ = ClimberConstants.STRAP_RETRACTED_POSITION;
      case RETRACTED:
        Pickup.getInstance().setPickupMode(PickupMode.CLIMB);
        io_.target_strap_rotations_ = ClimberConstants.STRAP_RETRACTED_POSITION;
        io_.strap_motor_target_offset += ClimberConstants.STRAP_SETPOINT_BUMP;
        break;
      default:
        break;
    }
  }

  public void backStage() {
    switch (io_.current_mode_) {
      case PRECLIMB:
        io_.current_mode_ = ClimberMode.DISABLED;
        // The elevators default cmd will set to stow
        Elastic.selectTab("Teleop");
        break;
      default:
        DriverStation.reportError("Can no longer return to previous stage", false);
        break;
    }
  }

  public void climbSetpoint() {
    io_.target_strap_rotations_ += ClimberConstants.STRAP_SETPOINT_BUMP;
  }

  public ClimberMode getMode() {
    return io_.current_mode_;
  }

  public boolean lockOutControl() {
    return io_.current_mode_ != ClimberMode.DISABLED;
  }

  public class ClimberPeriodicIo implements Logged {
    @Log.File public double target_strap_rotations_ = 0;
    @Log.File public double strap_motor_target_offset = 0;
    @Log.File public double strap_motor_current_ = 0;
    @Log.File public double prong_motor_demand = 0;
    @Log.File public double current_prong_rotations_ = 0;
    @Log.File public double target_prong_rotations_ = 0;
    @Log.File public double target_arm_output_ = 0;
    @Log.File public ClimberMode current_mode_ = ClimberMode.DISABLED;
    @Log.File public double deploying_start_time_ = 0;
  }

  @Override
  public Logged getLoggingObject() {
    return io_;
  }
}
