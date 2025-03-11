// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.lib.ElevatorTargets.Target;
import frc.mw_lib.command.LazyCommand;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Claw.ClawMode;
import frc.robot.subsystems.Claw.GamePiece;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.SpeedLimit;

public class CoralLoad extends LazyCommand {
  static Claw claw_;

  /** Creates a new ExampleLazyCommand. */
  public CoralLoad() {
    // Creates a new LazyCommand with the selected amount of seconds(double) to wait
    // before allowing
    // the lazy command to end
    super(0.5);
    claw_ = Claw.getInstance();
    addRequirements(claw_, Elevator.getInstance());
    setName(this.getClass().getSimpleName());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    claw_.setGamePiece(GamePiece.CORAL);
    claw_.setClawMode(ClawMode.LOAD);
    Elevator.getInstance().setTarget(Target.STATION);
    Elevator.getInstance().setSpeedLimit(SpeedLimit.CORAL);
    this.timerReset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    claw_.setClawMode(ClawMode.IDLE);
    Elevator.getInstance().setTarget(Target.STOW);
  }

  // unique to lazy command and serves the same purpose as "isFinished()" {*THE
  // COMMAND WILL END IF
  // THIS IS TRUE & THE TIME HAS ELAPSED*}
  @Override
  public boolean isConditionMet() {
    return Claw.getInstance().hasCoral();
  }
}
