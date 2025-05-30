// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.lib.ElevatorTargets.TargetType;
import frc.mw_lib.command.LazyCommand;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Claw.ClawMode;
import frc.robot.subsystems.Claw.GamePiece;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Pickup;
import frc.robot.subsystems.Pickup.PickupMode;

public class CoralLoad extends LazyCommand {
  /** Creates a new ExampleLazyCommand. */
  public CoralLoad() {
    // Creates a new LazyCommand with the selected amount of seconds(double) to wait
    // before allowing
    // the lazy command to end
    super(0.5);
    addRequirements(Claw.getInstance(), Elevator.getInstance());
    setName(this.getClass().getSimpleName());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Claw.getInstance().setGamePiece(GamePiece.CORAL);
    Claw.getInstance().setClawMode(ClawMode.LOAD);
    Elevator.getInstance().setTarget(TargetType.STATION);
    Pickup.getInstance().setPickupMode(PickupMode.STATION);
    this.timerReset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Claw.getInstance().setClawMode(ClawMode.IDLE);
    Elevator.getInstance().setTarget(TargetType.CORAL_INTAKE);
  }

  // unique to lazy command and serves the same purpose as "isFinished()" {*THE
  // COMMAND WILL END IF
  // THIS IS TRUE & THE TIME HAS ELAPSED*}
  @Override
  public boolean isConditionMet() {
    return Claw.getInstance().hasCoral();
  }
}
