// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.subsystem.Subsystem;
import frc.robot.Constants;
import monologue.Annotations.Log;
import monologue.Logged;

public class Claw extends Subsystem {

  private TalonFX clamp_motor_, wheel_motor_;
  private TalonFXConfiguration clamp_config;
  private TalonFXConfiguration wheel_config;

  ControlRequest clamp_request;

  public enum ClawMode {
    CLOSED,
    OPEN,
    SHOOT,
    LOAD,
    IDLE
  }

  // Singleton pattern
  private static Claw example_instance = null;

  // C
  public static Claw getInstance() {
    if (example_instance == null) {
      example_instance = new Claw();
    }
    return example_instance;
  }

  /** Class Members */
  private ClawPeriodicIo io_;

  private Claw() {
    // Create io object first in subsystem configuration
    io_ = new ClawPeriodicIo();

    clamp_motor_ = new TalonFX(Constants.ClawConstants.CLAMP_MOTOR_ID);
    wheel_motor_ = new TalonFX(Constants.ClawConstants.WHEEL_MOTOR_ID);
    clamp_request = new PositionVoltage(0).withSlot(0);
    clamp_config = new TalonFXConfiguration();
    wheel_config = new TalonFXConfiguration();

    wheel_config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    clamp_config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    clamp_config.CurrentLimits.SupplyCurrentLimit = Constants.ClawConstants.CLAMP_CURRENT_LIMIT;
    clamp_config.CurrentLimits.StatorCurrentLimitEnable = true;
    clamp_config.Feedback.SensorToMechanismRatio = Constants.ClawConstants.CLAMP_SENSOR_TO_MECHANISM_RATION;
    clamp_config.Feedback.FeedbackRotorOffset = Constants.ClawConstants.CLAMP_ZERO_OFFSET;
    clamp_config.Slot0 = Constants.ClawConstants.CLAMP_GAINS;

    clamp_motor_.getConfigurator().apply(clamp_config);
    wheel_motor_.getConfigurator().apply(wheel_config);

    SmartDashboard.putNumber("%Power", 0);

    // Call reset last in subsystem configuration
    reset();
  }

  /**
   * This function should be logic and code to fully reset your subsystem. This is
   * called during
   * initialization, and should handle I/O configuration and initializing data
   * members.
   */
  @Override
  public void reset() {

  }

  /**
   * Inside this function, all of the SENSORS should be read into variables stored
   * in the PeriodicIO
   * class defined below. There should be no calls to output to actuators, or any
   * logic within this
   * function.
   */
  @Override
  public void readPeriodicInputs(double timestamp) {
    io_.clamp_current_ = clamp_motor_.getSupplyCurrent().getValue().in(Amps);
    io_.current_clamp_angle_ = clamp_motor_.getPosition().getValue().in(Radians);
  }

  /**
   * Inside this function, all of the LOGIC should compute updates to output
   * variables in the
   * PeriodicIO class defined below. There should be no calls to read from sensors
   * or write to
   * actuators in this function.
   */
  @Override
  public void updateLogic(double timestamp) {
    switch (io_.claw_mode_) {
      case CLOSED:
        io_.target_clamp_angle = Constants.ClawConstants.CLOSED_ANGLE;
        clamp_request = new VoltageOut(Constants.ClawConstants.CLOSED_VOLTS);
        io_.wheel_output_ = 0;
        break;
      case SHOOT:
        io_.target_clamp_angle = Constants.ClawConstants.CLOSED_ANGLE;
        clamp_request = new VoltageOut(Constants.ClawConstants.CLOSED_VOLTS);
        io_.wheel_output_ = Constants.ClawConstants.WHEEL_SHOOT_SPEED;
        break;
      case OPEN:
        io_.target_clamp_angle = Constants.ClawConstants.OPEN_ANGLE;
        clamp_request = new PositionVoltage(0).withSlot(0).withPosition(io_.target_clamp_angle / (Math.PI * 2));
        io_.wheel_output_ = 0;
        break;
      case LOAD:
        io_.target_clamp_angle = Constants.ClawConstants.LOAD_ANGLE;
        clamp_request = new PositionVoltage(0).withSlot(0).withPosition(io_.target_clamp_angle / (Math.PI * 2));
        io_.wheel_output_ = Constants.ClawConstants.WHEEL_LOAD_SPEED;
        break;
      case IDLE:
      default:
        io_.target_clamp_angle = Constants.ClawConstants.CLOSED_ANGLE;
        clamp_request = new VoltageOut(0);
        io_.wheel_output_ = 0;
        break;
    }
  }

  /**
   * Inside this function actuator OUTPUTS should be updated from data contained
   * in the PeriodicIO
   * class defined below. There should be little to no logic contained within this
   * function, and no
   * sensors should be read.
   */
  @Override
  public void writePeriodicOutputs(double timestamp) {
    wheel_motor_.set(io_.wheel_output_);
    clamp_motor_.setControl(clamp_request);
    // clamp_motor_.set(SmartDashboard.getNumber("%Power", 0));
  }

  /**
   * Inside this function telemetry should be output to smartdashboard. The data
   * should be collected
   * out of the PeriodicIO class instance defined below. There should be no sensor
   * information read
   * in this function nor any outputs made to actuators within this function. Only
   * publish to
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

  public class ClawPeriodicIo implements Logged {
    @Log.File
    public ClawMode claw_mode_ = ClawMode.IDLE;
    @Log.File
    public double wheel_output_ = 0;
    @Log.File
    public double clamp_current_ = 0;
    @Log.File
    public double current_clamp_angle_ = 0.0;
    @Log.File
    public double target_clamp_angle = 0.0;
  }

  public double getClampAngle() {
    return io_.current_clamp_angle_;
  }

  public void setClawMode(ClawMode claw_mode) {
    io_.claw_mode_ = claw_mode;
  }

  public double getClampAmps() {
    return io_.clamp_current_;
  }

  // public void resetClampAngle() {
  // clamp_motor_.setPosition(0);
  // }

  @Override
  public Logged getLoggingObject() {
    return io_;
  }
}
