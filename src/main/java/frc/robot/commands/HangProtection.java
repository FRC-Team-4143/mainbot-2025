// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.ElevatorTargets.Target;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.SpeedLimit;

public class HangProtection extends Command {

  private boolean was_unsafe;
  private boolean has_hit_saftey = false; 

  /** Creates a new CoralEject. */
  public HangProtection() {
    addRequirements(Elevator.getInstance());
    setName(this.getClass().getSimpleName());
    //TODO: Still need some way to prevent other cmds interrupting this one
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    was_unsafe = Elevator.getInstance().isInL4DangerZone();
    has_hit_saftey = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (was_unsafe) {
      Elevator.getInstance().setSpeedLimit(SpeedLimit.SAFTEY);
      Elevator.getInstance().setTarget(Target.L4_SAFETY);
      if (Elevator.getInstance().isElevatorAndArmAtTarget()) {
        has_hit_saftey = true;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //default cmd will set stow
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (was_unsafe == false) {
      return true;
    } else {
      if (has_hit_saftey) {
        return true;
      }
    }
    return false;
  }
}
