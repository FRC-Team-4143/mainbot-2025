// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.SpeedLimit;

public class HangProtection extends Command {

  private boolean was_unsafe;

  // State before engaging safety
  private SpeedLimit savedSpeed;
  private int num_intermediates = 0;

  /** Creates a new CoralEject. */
  public HangProtection() {
    addRequirements(Elevator.getInstance());
    setName(this.getClass().getSimpleName());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    savedSpeed = Elevator.getInstance().getCurrSpeedLimit();
    was_unsafe = Elevator.getInstance().isArmInDangerZone();
    num_intermediates = Elevator.getInstance().getNumIntermediates();

    if (was_unsafe) {
      Elevator.getInstance().setSpeedLimit(SpeedLimit.SAFETY);
      Elevator.getInstance().addSafetyIntermediate();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Elevator.getInstance().setSpeedLimit(savedSpeed);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // We are done if we were either not unsafe, or the safety intermediate is gone
    if (!was_unsafe || Elevator.getInstance().getNumIntermediates() == num_intermediates) {
      return true;
    }
    return false;
  }
}
