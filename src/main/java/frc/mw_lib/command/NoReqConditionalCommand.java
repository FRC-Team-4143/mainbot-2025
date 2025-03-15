// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.mw_lib.command;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.util.function.BooleanSupplier;

public class NoReqConditionalCommand extends Command {

  private Command command_A;
  private Command command_B;
  private BooleanSupplier supplier;
  private Command selected_command;

  /** Creates a new CoralEject. */
  public NoReqConditionalCommand(Command a, Command b, BooleanSupplier s) {
    command_A = a;
    command_B = b;
    supplier = s;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    selected_command = supplier.getAsBoolean() ? command_A : command_B;
    CommandScheduler.getInstance().schedule(selected_command);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Do nothing intentionally
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    CommandScheduler.getInstance().cancel(selected_command);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return selected_command.isFinished();
  }
}
