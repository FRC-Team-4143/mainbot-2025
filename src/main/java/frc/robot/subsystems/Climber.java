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
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.mw_lib.logging.Elastic;
import frc.mw_lib.subsystem.Subsystem;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;
import monologue.Annotations.Log;
import monologue.Logged;

public class Climber extends Subsystem {

  private TalonFX strap_motor_;
  private ControlRequest strap_request_;
  private PositionVoltage strap_position_request_;
  private VoltageOut strap_voltage_request_;
  private TalonFXConfiguration strap_config_;

  private Counter prong_counter_;
  private PIDController prong_controller_;
  private Spark prong_motor_;

  private Spark arm_motor_;

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

    strap_motor_ = new TalonFX(Constants.ClimberConstants.STRAP_ID);
    prong_motor_ = new Spark(Constants.ClimberConstants.PRONG_ID);
    prong_motor_.setInverted(true);
    arm_motor_ = new Spark(Constants.ClimberConstants.ARM_ID);
    arm_motor_.setInverted(true);

    strap_motor_.getConfigurator().apply(Constants.ClimberConstants.STRAP_GAINS);

    strap_position_request_ = new PositionVoltage(0.0);
    strap_position_request_.withSlot(0);
    strap_voltage_request_ = new VoltageOut(0);

    strap_config_ = new TalonFXConfiguration();
    strap_config_.MotorOutput.Inverted = Constants.ClimberConstants.STRAP_INVERSION;
    strap_config_.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    prong_counter_ = new Counter(Constants.ClimberConstants.PRONG_COUNTER_ID);
    prong_controller_ =
        new PIDController(
            Constants.ClimberConstants.PRONG_P, 0, Constants.ClimberConstants.PRONG_D);

    reset();
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
  public void readPeriodicInputs(double timestamp) {}

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
        prong_controller_.setSetpoint(Constants.ClimberConstants.PRONG_PRESET_COUNT);
        double vcomp = 11.0 / RobotController.getBatteryVoltage(); // Tuned at 11.0v
        io_.prong_motor_target = prong_controller_.calculate(prong_counter_.get()) * vcomp;

        if (prong_counter_.get() >= Constants.ClimberConstants.PRONG_PRESET_COUNT) {
          io_.current_mode_ = ClimberMode.PRESET;
        }
        break;
      case PRESET:
        io_.prong_motor_target = Constants.ClimberConstants.PRONG_HOLD_SPEED;
        // wait
        break;
      case DEPLOYING:
        io_.prong_motor_target = Constants.ClimberConstants.PRONG_DEPLOY_SPEED;
        io_.arm_motor_target = Constants.ClimberConstants.ARM_DEPLOY_SPEED;
        if (io_.deploying_start_time_ == 0) {
          io_.deploying_start_time_ = timestamp;
        }
        if (timestamp - io_.deploying_start_time_ >= Constants.ClimberConstants.DEPLOYING_TIME) {
          io_.current_mode_ = ClimberMode.DEPLOYED;
        }
        break;
      case DEPLOYED:
        io_.arm_motor_target = Constants.ClimberConstants.ARM_HOLD_SPEED;
        io_.prong_motor_target = Constants.ClimberConstants.PRONG_DEPLOY_SPEED;
        // Wait for transition
        break;
      case RETRACTED:
        io_.arm_motor_target = 0;
        io_.prong_motor_target = Constants.ClimberConstants.PRONG_DEPLOY_SPEED;
        strap_request_ = strap_position_request_.withPosition(io_.strap_motor_target);
        break;
      case DISABLED:
      default:
        // do nothing
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
    prong_motor_.set(io_.prong_motor_target);
    arm_motor_.set(io_.arm_motor_target);
  }

  /**
   * Inside this function telemetry should be output to smartdashboard. The data should be collected
   * out of the PeriodicIO class instance defined below. There should be no sensor information read
   * in this function nor any outputs made to actuators within this function. Only publish to
   * smartdashboard here.
   */
  @Override
  public void outputTelemetry(double timestamp) {
    SmartDashboard.putNumber("Subsystems/Climber/strap_motor_target", io_.strap_motor_target);
    SmartDashboard.putNumber("Subsystems/Climber/prong_motor_target", io_.prong_motor_target);
    SmartDashboard.putNumber("Subsystems/Climber/arm_motor_target", io_.arm_motor_target);
    SmartDashboard.putString("Subsystems/Climber/current_mode_", io_.current_mode_.toString());
    SmartDashboard.putNumber("Subsystems/Climber/prong_count", prong_counter_.get());
  }

  public void nextStage() {
    switch (io_.current_mode_) {
      case DISABLED:
        prong_counter_.reset();
        io_.current_mode_ = ClimberMode.PRECLIMB;
        // The elevators default cmd will set to climb
        Elastic.selectTab("Climb");
        break;
      case PRECLIMB:
        io_.current_mode_ = ClimberMode.STAGING;
        break;
      case PRESET:
        io_.current_mode_ = ClimberMode.DEPLOYING;
        break;
      case DEPLOYED:
        io_.current_mode_ = ClimberMode.RETRACTED;
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
    if (io_.current_mode_ == ClimberMode.RETRACTED) {
      io_.strap_motor_target += ClimberConstants.STRAP_SETPOINT_BUMP;
    }
  }

  public ClimberMode getMode() {
    return io_.current_mode_;
  }

  public boolean lockOutControl() {
    return io_.current_mode_ != ClimberMode.DISABLED;
  }

  public class ClimberPeriodicIo implements Logged {
    @Log.File public double strap_motor_target = 0;
    @Log.File public double strap_motor_target_offset = 0;
    @Log.File public double prong_motor_target = 0;
    @Log.File public double arm_motor_target = 0;
    @Log.File public ClimberMode current_mode_ = ClimberMode.DISABLED;
    @Log.File public double deploying_start_time_ = 0;
  }

  @Override
  public Logged getLoggingObject() {
    return io_;
  }
}
