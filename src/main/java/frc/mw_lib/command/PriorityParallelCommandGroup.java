// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.mw_lib.command;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class PriorityParallelCommandGroup extends Command {

  private Command parallel_command_;
  private Command priority_command_;

  public PriorityParallelCommandGroup(Command parallel_command, Command priority_command) {
    parallel_command_ = parallel_command;
    priority_command_ = priority_command;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    CommandScheduler.getInstance().schedule(parallel_command_);
    CommandScheduler.getInstance().schedule(priority_command_);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    CommandScheduler.getInstance().cancel(parallel_command_);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return priority_command_.isFinished();
  }
}
