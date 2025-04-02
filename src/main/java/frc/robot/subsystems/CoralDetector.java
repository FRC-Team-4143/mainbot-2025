// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.mw_lib.subsystem.Subsystem;
import monologue.Logged;

public class CoralDetector extends Subsystem {

  // Singleton pattern
  private static CoralDetector coral_detector_instance_ = null;

  public static CoralDetector getInstance() {
    if (coral_detector_instance_ == null) {
      coral_detector_instance_ = new CoralDetector();
    }
    return coral_detector_instance_;
  }

  /** Class Members */
  private CoralDetectorPeriodicIo io_;

  private CoralDetector() {
    // Create io object first in subsystem configuration
    io_ = new CoralDetectorPeriodicIo();

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
  public void readPeriodicInputs(double timestamp) {}

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

  public class CoralDetectorPeriodicIo implements Logged {}

  public Logged getLoggingObject() {
    return io_;
  }
}
