package frc.lib;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class LazyCommand extends Command {

  private double target_elapsed_milliseconds_;

  public Timer command_alarm_ = new Timer();

  // ^ named by J. A. H. (based on alarm clock)

  /**
   * @param milliseconds the alarm that allows for commands to finish after a given amount of time
   *     (in milliseconds)
   */
  public LazyCommand(double milliseconds) {

    super();

    target_elapsed_milliseconds_ = milliseconds;
  }

  public void timerReset(){
    command_alarm_.reset();
    command_alarm_.start();
  }

  /**
   * @return Return a boolean based on if the target elapsed milliseconds has been met
   */
  public boolean isAlarmFinished() {

    if (command_alarm_.hasElapsed(target_elapsed_milliseconds_ / 1000)) {
      command_alarm_.stop();
      return true;
    }
    return false;
  }

  /**
   * Be sure to call the method "timerReset()" in initialize
   */
  @Override
  public void initialize() {
  }

  @Override
  public boolean isFinished() {

    return isAlarmFinished() && isConditionMet();
  }

  public boolean isConditionMet() {

    return true;
  }
}
