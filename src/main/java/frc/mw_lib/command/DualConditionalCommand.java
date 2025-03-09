// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.mw_lib.command;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.function.BooleanSupplier;

public class DualConditionalCommand extends Command {

  private Command command_A;
  private Command command_B;
  private BooleanSupplier supplier_A;
  private BooleanSupplier supplier_B;
  private boolean isCommand_A_Bool;
  private boolean isCommand_B_Bool;

  /**
   * Creates a new CoralEject.
   *
   * @param a Command a (pritorized over command b)
   * @param b Command b
   * @param sa boolean supplier for Command a
   * @param sb boolean supplier for Command b
   */
  public DualConditionalCommand(Command a, Command b, BooleanSupplier sa, BooleanSupplier sb) {
    command_A = a;
    command_B = b;
    supplier_A = sa;
    supplier_B = sb;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isCommand_A_Bool = supplier_A.getAsBoolean();
    isCommand_B_Bool = supplier_B.getAsBoolean();
    if (isCommand_A_Bool) {
      for (Subsystem s : command_A.getRequirements()) {
        Command command = CommandScheduler.getInstance().requiring(s);
        if (command != null) CommandScheduler.getInstance().cancel(command);
      }
      command_A.initialize();
      System.out.println("Initialized Command BARGE");
    } else if (isCommand_B_Bool) {
      for (Subsystem s : command_B.getRequirements()) {
        Command command = CommandScheduler.getInstance().requiring(s);
        if (command != null) CommandScheduler.getInstance().cancel(command);
      }
      command_B.initialize();
      System.out.println("Initialized Command PROCESSOR");
    } else {
      this.end(true);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (isCommand_A_Bool) {
      command_A.execute();
    } else if (isCommand_B_Bool) {
      command_B.execute();
    } else {
      // do nothing
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (isCommand_A_Bool) {
      command_A.end(interrupted);
    } else if (isCommand_B_Bool) {
      command_B.end(interrupted);
    } else {
      // end outer command
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (isCommand_A_Bool) {
      return command_A.isFinished();
    } else if (isCommand_B_Bool) {
      return command_B.isFinished();
    }
    return true;
  }
}
