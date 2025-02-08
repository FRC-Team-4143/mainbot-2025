// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.mw_lib.command.LazyCommand;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Claw.ClawMode;
import frc.robot.subsystems.Claw.GamePiece;
import frc.robot.subsystems.Elevator;

public class AlgaeLoad extends LazyCommand {
  static Claw claw_;
  static Elevator elevator_;

  /** Creates a new ExampleLazyCommand. */
  public AlgaeLoad() {
    // Creates a new LazyCommand with the selected amount of seconds(double) to wait before allowing
    // the lazy command to end
    super(0.02);
    claw_ = Claw.getInstance();
    addRequirements(claw_);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    claw_.setGamePiece(GamePiece.ALGAE);
    claw_.setClawMode(ClawMode.LOAD);

    // Allows for the timer to reset properly and for the delay in the lazy command
    // to be timed correctly
    this.timerReset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    claw_.setClawMode(ClawMode.IDLE);
  }

  // unique to lazy command and serves the same purpose as "isFinished()" {*THE COMMAND WILL END IF
  // THIS IS TRUE & THE TIME HAS ELAPSED*}
  @Override
  public boolean isConditionMet() {
    return true;
  }
}
