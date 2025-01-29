// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.mw_lib.subsystem.Subsystem;
import frc.robot.Constants.ClawConstants;
import monologue.Annotations.Log;
import monologue.Logged;

public class Claw extends Subsystem {

  private TalonFX clamp_motor_, wheel_motor_;
  private TalonFXConfiguration clamp_config_, wheel_config_;
  private ControlRequest current_clamp_request_;
  private VoltageOut voltage_clamp_request_;
  private PositionVoltage position_clamp_request_;

  public enum ClawMode {
    CLOSED,
    OPEN,
    SHOOT,
    LOAD,
    IDLE
  }

  // Singleton pattern
  private static Claw claw_instance = null;

  // C
  public static Claw getInstance() {
    if (claw_instance == null) {
      claw_instance = new Claw();
    }
    return claw_instance;
  }

  /** Class Members */
  private ClawPeriodicIo io_;

  private Claw() {
    // Create io object first in subsystem configuration
    io_ = new ClawPeriodicIo();

    clamp_motor_ = new TalonFX(ClawConstants.CLAMP_MOTOR_ID);
    wheel_motor_ = new TalonFX(ClawConstants.WHEEL_MOTOR_ID);
    voltage_clamp_request_ = new VoltageOut(0);
    position_clamp_request_ = new PositionVoltage(0).withSlot(0);
    current_clamp_request_ = voltage_clamp_request_;
    clamp_config_ = new TalonFXConfiguration();
    wheel_config_ = new TalonFXConfiguration();

    wheel_config_.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    clamp_config_.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    clamp_config_.CurrentLimits.SupplyCurrentLimit = ClawConstants.CLAMP_CURRENT_LIMIT;
    clamp_config_.CurrentLimits.StatorCurrentLimitEnable = true;
    clamp_config_.Feedback.SensorToMechanismRatio = ClawConstants.CLAMP_SENSOR_TO_MECHANISM_RATION;
    clamp_config_.Feedback.FeedbackRotorOffset = ClawConstants.CLAMP_ZERO_OFFSET;
    clamp_config_.Slot0 = ClawConstants.CLAMP_GAINS;

    clamp_motor_.getConfigurator().apply(clamp_config_);
    wheel_motor_.getConfigurator().apply(wheel_config_);

    // Call reset last in subsystem configuration
    reset();
  }

  /**
   * This function should be logic and code to fully reset your subsystem. This is called during
   * initialization, and should handle I/O configuration and initializing data members.
   */
  @Override
  public void reset() {}

  /**
   * Inside this function, all of the SENSORS should be read into variables stored in the PeriodicIO
   * class defined below. There should be no calls to output to actuators, or any logic within this
   * function.
   */
  @Override
  public void readPeriodicInputs(double timestamp) {
    io_.clamp_current_ = clamp_motor_.getSupplyCurrent().getValue().in(Amps);
    io_.current_clamp_angle_ = clamp_motor_.getPosition().getValue().in(Radians);
  }

  /**
   * Inside this function, all of the LOGIC should compute updates to output variables in the
   * PeriodicIO class defined below. There should be no calls to read from sensors or write to
   * actuators in this function.
   */
  @Override
  public void updateLogic(double timestamp) {
    switch (io_.claw_mode_) {
      case CLOSED:
        io_.target_clamp_angle = ClawConstants.CLOSED_ANGLE;
        current_clamp_request_ = voltage_clamp_request_.withOutput(ClawConstants.CLOSED_VOLTS);
        io_.wheel_output_ = 0;
        break;
      case SHOOT:
        io_.target_clamp_angle = ClawConstants.CLOSED_ANGLE;
        current_clamp_request_ = voltage_clamp_request_.withOutput(ClawConstants.CLOSED_VOLTS);
        io_.wheel_output_ = ClawConstants.WHEEL_SHOOT_SPEED;
        break;
      case OPEN:
        io_.target_clamp_angle = ClawConstants.OPEN_ANGLE;
        current_clamp_request_ =
            position_clamp_request_.withPosition(Units.radiansToRotations(io_.target_clamp_angle));
        io_.wheel_output_ = 0;
        break;
      case LOAD:
        io_.target_clamp_angle = ClawConstants.LOAD_ANGLE;
        current_clamp_request_ =
            position_clamp_request_.withPosition(Units.radiansToRotations(io_.target_clamp_angle));
        io_.wheel_output_ = ClawConstants.WHEEL_LOAD_SPEED;
        break;
      case IDLE:
        current_clamp_request_ = voltage_clamp_request_.withOutput(0);
        io_.wheel_output_ = 0;
        break;
      default:
        io_.target_clamp_angle = ClawConstants.CLOSED_ANGLE;
        current_clamp_request_ = voltage_clamp_request_.withOutput(ClawConstants.CLOSED_VOLTS);
        io_.wheel_output_ = 0;
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
    wheel_motor_.set(io_.wheel_output_);
    clamp_motor_.setControl(current_clamp_request_);
  }

  /**
   * Inside this function telemetry should be output to smartdashboard. The data should be collected
   * out of the PeriodicIO class instance defined below. There should be no sensor information read
   * in this function nor any outputs made to actuators within this function. Only publish to
   * smartdashboard here.
   */
  @Override
  public void outputTelemetry(double timestamp) {
    SmartDashboard.putNumber("Clamp Angle", io_.current_clamp_angle_);
    SmartDashboard.putNumber("Target Angle", io_.target_clamp_angle);
    SmartDashboard.putNumber("Clamp Amps", io_.clamp_current_);
    SmartDashboard.putNumber("Clamp Volts", clamp_motor_.getMotorVoltage().getValue().in(Volts));
    SmartDashboard.putString("Clamp Mode", io_.claw_mode_.toString());
  }

  /**
   * @return the current angle of the clamp in radians
   */
  public double getClampAngle() {
    return io_.current_clamp_angle_;
  }

  /**
   * Set the mode of the claw
   *
   * @param claw_mode the new mode to be set
   */
  public void setClawMode(ClawMode claw_mode) {
    io_.claw_mode_ = claw_mode;
  }

  /**
   * @return The supply current in Amps
   */
  public double getClampAmps() {
    return io_.clamp_current_;
  }

  public class ClawPeriodicIo implements Logged {
    @Log.File public ClawMode claw_mode_ = ClawMode.IDLE;
    @Log.File public double wheel_output_ = 0;
    @Log.File public double clamp_current_ = 0;
    @Log.File public double current_clamp_angle_ = 0.0;
    @Log.File public double target_clamp_angle = 0.0;
  }

  @Override
  public Logged getLoggingObject() {
    return io_;
  }
}
