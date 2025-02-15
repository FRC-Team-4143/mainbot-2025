package frc.mw_lib.subsystem;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import frc.mw_lib.logging.GitLogger;

import java.util.ArrayList;
import monologue.Annotations.Log;
import monologue.Logged;
import monologue.Monologue;

public abstract class SubsystemManager {

  // Supposedly 1 is a good starting point, but can increase if we have issues
  private static final int START_THREAD_PRIORITY = 99;

  public class Contain implements Logged {
    @Log.File public ArrayList<Logged> subsystems_ios = new ArrayList<>();
  }

  protected ArrayList<Subsystem> subsystems;
  protected Notifier loopThread;
  protected boolean log_init = false;
  protected Contain ios;

  public SubsystemManager() {
    // Initialize the subsystem list
    subsystems = new ArrayList<>();

    // create the thread to loop the subsystems and mark as daemon thread so
    // the robot program can properly stop
    loopThread = new Notifier(this::doControlLoop);

    ios = new Contain();

    // Logger
  }

  public void registerSubsystem(Subsystem system) {
    subsystems.add(system);
    ios.subsystems_ios.add(system.getLoggingObject());
  }

  private void doControlLoop() {

    // For each subsystem get incoming data
    double timestamp = Timer.getFPGATimestamp();
    for (Subsystem subsystem : subsystems) {
      try {
        subsystem.readPeriodicInputs(timestamp);
      } catch (Exception e) {
        e.printStackTrace();
        DataLogManager.log(subsystem.getClass().getCanonicalName() + "failed to read inputs");
      }
    }

    // Now update the logic for each subsystem to allow I/O to relax
    timestamp = Timer.getFPGATimestamp();
    for (Subsystem subsystem : subsystems) {
      try {
        subsystem.updateLogic(timestamp);
      } catch (Exception e) {
        e.printStackTrace();
        DataLogManager.log(subsystem.getClass().getCanonicalName() + "failed to update logic");
      }
    }

    // Finally write the outputs to the actuators
    timestamp = Timer.getFPGATimestamp();
    for (Subsystem subsystem : subsystems) {
      try {
        subsystem.writePeriodicOutputs(timestamp);
      } catch (Exception e) {
        e.printStackTrace();
        DataLogManager.log(subsystem.getClass().getCanonicalName() + "failed to write outputs");
      }
    }

    // Run the logger!
    if (log_init && DriverStation.isEnabled()) {
      runLog(Timer.getFPGATimestamp());
    }
  }

  /** Completes the subsystem registration process and begins calling each subsystem in a loop */
  protected void completeRegistration() {
    loopThread.startPeriodic(.01);

    Monologue.setupMonologue(ios, "Robot", true, false);
    DriverStation.startDataLog(DataLogManager.getLog());
    GitLogger.logGitData();
    GitLogger.putGitDataToDashboarad();
    log_init = true;
  }

  /**
   * Helper function to collect log data classes for recording
   *
   * @param timestamp the timestamp logging was started at from the FPGA
   */
  protected void runLog(double timestamp) {
    // If it is valid, collect the subsystem I/Os
    ios.subsystems_ios.clear();
    for (Subsystem subsystem : subsystems) {
      ios.subsystems_ios.add(subsystem.getLoggingObject());
    }

    try {
      Monologue.updateAll();
    } catch (Exception e) {
      DataLogManager.log("Monologue failed to log io");
    }
  }

  /**
   * When ready to put telemetry out to smartdashboard, call this function to trigger each subsystem
   * to publish its held data. This is supposed to be called by robotPeriodic so telemetry is output
   * in any mode.
   */
  public void outputTelemetry() {
    for (Subsystem subsystem : subsystems) {
      subsystem.outputTelemetry(Timer.getFPGATimestamp());
    }
  }

  /**
   * If subsystems all need to be reset before a robot mode change, call this function to cleanly
   * handle resetting them together. If only one subsystem needs to be reset, that can be accessed
   * through the getInstance method.
   */
  public void reset() {
    for (Subsystem subsystem : subsystems) {
      subsystem.reset();
    }
  }
}
