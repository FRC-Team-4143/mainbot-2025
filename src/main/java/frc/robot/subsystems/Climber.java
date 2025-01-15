// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.AnalogInput;
import frc.lib.subsystem.Subsystem;
import monologue.Annotations.Log;
import monologue.Logged;

public class Climber extends Subsystem {

  enum climberStage {
    RETRACTED,
    CLAMPED,
    EXTENDED,
    AIRBORNE
  }

  TalonFX climber_motor_ = new TalonFX(0);
  TalonFX latch = new TalonFX(1);
  AnalogInput climber_encoder_ = new AnalogInput(0);
  // Singleton pattern
  private static Climber example_instance = null;

  // C
  public static Climber getInstance() {
    if (example_instance == null) {
      example_instance = new Climber();
    }
    return example_instance;
  }

  /** Class Members */
  private ClimberPeriodicIo io_;

  private Climber() {
    // Create io object first in subsystem configuration
    io_ = new ClimberPeriodicIo();

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
    // read climber motor
    // ./climber_encoder_.getValue();
  }

  /**
   * Inside this function, all of the LOGIC should compute updates to output variables in the
   * PeriodicIO class defined below. There should be no calls to read from sensors or write to
   * actuators in this function.
   */
  @Override
  public void updateLogic(double timestamp) {}

  /**
   * Inside this function actuator OUTPUTS should be updated from data contained in the PeriodicIO
   * class defined below. There should be little to no logic contained within this function, and no
   * sensors should be read.
   */
  @Override
  public void writePeriodicOutputs(double timestamp) {}

  /**
   * Inside this function telemetry should be output to smartdashboard. The data should be collected
   * out of the PeriodicIO class instance defined below. There should be no sensor information read
   * in this function nor any outputs made to actuators within this function. Only publish to
   * smartdashboard here.
   */
  @Override
  public void outputTelemetry(double timestamp) {}

  public class ClimberPeriodicIo implements Logged {
    @Log.File private double climber_y_pose_;
    @Log.File private double climber_rotational_pose_;
    @Log.File private double clamper_pose_;
    @Log.File private double clamper_current_;
    @Log.File private double climber_vertical_target;
    @Log.File private double climber_rotational_target;
    @Log.File private double clamper_target;
  }

  @Override
  public Logged getLoggingObject() {
    return io_;
  }
}
