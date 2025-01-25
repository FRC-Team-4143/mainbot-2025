package frc.mw_lib.command;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class LazyCommand extends Command {

  private double target_elapsed_seconds_;

  private Timer command_alarm_ = new Timer();

  // ^ named by Jacob Haley (based on alarm clock)

  /**
   * @param seconds the alarm that allows for commands to finish after a given amount of time (in
   *     seconds)
   */
  public LazyCommand(double seconds) {
    super();
    target_elapsed_seconds_ = seconds;
  }

  /**
   * This method is what resets the timer to 0.0 seconds whenever this is called {*THIS IS TO ADJUST
   * THE TIMER TO CORRECTLY TIME THE LAZY COMMAND*}
   */
  public void timerReset() {
    command_alarm_.reset();
    command_alarm_.start();
  }

  /**
   * @return Return a boolean based on if the target elapsed seconds has been met
   */
  public boolean isAlarmFinished() {

    if (command_alarm_.hasElapsed(target_elapsed_seconds_)) {
      command_alarm_.stop();
      return true;
    }
    return false;
  }

  @Override
  public boolean isFinished() {

    return isAlarmFinished() && isConditionMet();
  }

  /**
   * @return Returns a boolean based on if the condition for the lazy command is true, unique to
   *     lazy command and serves the same purpose as "isFinished()" {*THE COMMAND WILL END IF THIS
   *     IS TRUE & THE TIME HAS ELAPSED*}
   */
  public boolean isConditionMet() {

    return true;
  }
}
