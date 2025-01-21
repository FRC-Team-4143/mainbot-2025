// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib;

public class ExampleLazyCommand extends LazyCommand {
  /** Creates a new ExampleLazyCommand. */
  public ExampleLazyCommand() {
    // Creates a new LazyCommand with the selected amount of seconds(double) to wait before allowing
    // the lazy command to end
    super(0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    // Allows for the timer to reset properly and for the delay in the lazy command
    // to be timed correctly
    this.timerReset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isConditionMet() {

    return true;
  }
}
