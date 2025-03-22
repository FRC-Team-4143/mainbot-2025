// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.mw_lib.command;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.util.ArrayList;

public class NoReqSequentialCommandGroup extends Command {

  private ArrayList<Command> sequence = new ArrayList<>();
  private int index = 0;

  public NoReqSequentialCommandGroup() {}

  public NoReqSequentialCommandGroup(Command... commands) {
    addCommands(commands);
  }

  public void addCommands(Command... commands) {
    for (Command command : commands) {
      sequence.add(command);
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    index = 0;
    CommandScheduler.getInstance().schedule(sequence.get(index));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!sequence.get(index).isScheduled()) {
      index++;
      CommandScheduler.getInstance().schedule(sequence.get(index));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return index == (sequence.size() - 1) && !sequence.get(sequence.size() - 1).isScheduled();
  }
}
