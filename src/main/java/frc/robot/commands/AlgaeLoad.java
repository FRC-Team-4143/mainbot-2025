// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Claw.ClawMode;
import frc.robot.subsystems.Claw.GamePiece;

public class AlgaeLoad extends Command {
  static Claw claw_;

  public AlgaeLoad() {
    claw_ = Claw.getInstance();
    addRequirements(claw_);
    setName(this.getClass().getSimpleName());
  }

  @Override
  public void initialize() {
    claw_.setGamePiece(GamePiece.ALGAE);
    claw_.setClawMode(ClawMode.LOAD);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    claw_.setClawMode(ClawMode.IDLE);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
