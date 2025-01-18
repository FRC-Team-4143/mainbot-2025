// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radian;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.subsystem.Subsystem;
import frc.robot.Constants;
import frc.robot.Constants.ClawConstants;
import monologue.Annotations.Log;
import monologue.Logged;

public class Claw extends Subsystem {

  private TalonFX clamp_motor_, clamp_wheel_motor_;

  public enum ClawMode {
    CLOSED,
    OPEN,
    SQUEEZE,
    SHOOT,
    LOAD
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

    clamp_motor_ = new TalonFX(ClawConstants.CLAMP_CLAW_MOTOR_);
    clamp_wheel_motor_ = new TalonFX(ClawConstants.CLAMP_WHEEL_MOTOR_);

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
    io_.current_clamp_angle_ = clamp_motor_.getPosition().getValue().in(Radian);
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

    if (io_.clamp_current_ >= Constants.ClawConstants.CLAW_AMP_THRESHOLD && io_.claw_mode_ == ClawMode.CLOSED) {
      io_.claw_mode_ = ClawMode.SQUEEZE;
    }

    switch (io_.claw_mode_) {
      case CLOSED:
        io_.claw_output_ = Constants.ClawConstants.CLOSING_CLAW_SPEED;
        io_.wheel_output_ = 0;
        break;
      case SQUEEZE:
        io_.claw_output_ = Constants.ClawConstants.SQUEEZING_CLAW_SPEED;
        io_.wheel_output_ = 0;
        break;
      case SHOOT:
        // io_.claw_output_ = Constants.ClawConstants.CLOSED_CLAW_ANGLE;
        // io_.wheel_output_ = Constants.ClawConstants.CLAW_WHEEL_MAX_SPEED;
        break;
      case OPEN:
      default:
        io_.claw_output_ = Constants.ClawConstants.OPEN_CLAW_ANGLE;
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
    // clamp_wheel_motor_.set(io_.wheel_output_);
    clamp_motor_.set(io_.claw_output_);
    clamp_motor_.set(io_.wheel_output_);
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
    SmartDashboard.putNumber("Current", io_.clamp_current_);
  }

  public class ClawPeriodicIo implements Logged {
    @Log.File
    public ClawMode claw_mode_ = ClawMode.OPEN;
    @Log.File
    public double claw_output_ = 0;
    @Log.File
    public double wheel_output_ = 0;
    @Log.File
    public double clamp_current_ = 0;
    @Log.File
    public double current_clamp_angle_ = 0.0;

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

  @Override
  public Logged getLoggingObject() {
    return io_;
  }
}
