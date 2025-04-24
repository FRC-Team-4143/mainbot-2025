// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.ElevatorTargets.TargetType;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.claw.Claw.ClawMode;
import frc.robot.subsystems.claw.Claw.GamePiece;
import frc.robot.subsystems.elevator.Elevator;

public class AutoBargeSequence extends Command {
  /** Creates a new AutoBargeSequence. */
  public AutoBargeSequence() {
    addRequirements(Elevator.getInstance());
    setName(this.getClass().getSimpleName());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Claw.getInstance().setGamePiece(GamePiece.ALGAE);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Elevator.getInstance().isElevatorAndArmAtTarget()) {
      Claw.getInstance().setClawMode(ClawMode.SHOOT);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Claw.getInstance().setClawMode(ClawMode.IDLE);
    Elevator.getInstance().setTarget(TargetType.ALGAE_STOW);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !Claw.getInstance().hasAlgae();
  }
}
