// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.mw_lib.command;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.function.BooleanSupplier;

public class NoReqConditionalCommand extends Command {

  private Command command_A;
  private Command command_B;
  private BooleanSupplier supplier;
  private boolean isCommand_A;

  /** Creates a new CoralEject. */
  public NoReqConditionalCommand(Command a, Command b, BooleanSupplier s) {
    command_A = a;
    command_B = b;
    supplier = s;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isCommand_A = supplier.getAsBoolean();
    if (isCommand_A) {
      for (Subsystem s : command_A.getRequirements()) {
        Command command = CommandScheduler.getInstance().requiring(s);
        if (command != null) CommandScheduler.getInstance().cancel(command);
      }
      command_A.initialize();
    } else {
      for (Subsystem s : command_B.getRequirements()) {
        Command command = CommandScheduler.getInstance().requiring(s);
        if (command != null) CommandScheduler.getInstance().cancel(command);
      }
      command_B.initialize();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (isCommand_A) {
      command_A.execute();
    } else {
      command_B.execute();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (isCommand_A) {
      command_A.end(interrupted);
    } else {
      command_B.end(interrupted);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (isCommand_A) {
      return command_A.isFinished();
    } else {
      return command_B.isFinished();
    }
  }
}
