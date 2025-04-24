// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.ElevatorTargets.TargetType;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.claw.Claw.GamePiece;
import frc.robot.subsystems.elevator.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ManualElevatorOverride extends Command {

  public enum Level {
    L1,
    L2,
    L3,
    L4
  }

  private static GamePiece lastMode;
  private Level level_;

  /** Creates a new LowButtonCommand. */
  public ManualElevatorOverride(Level level) {
    level_ = level;
    addRequirements(Elevator.getInstance());
    setName(this.getClass().getSimpleName());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lastMode = Claw.getInstance().getGamePieceMode();

    if (Claw.getInstance().isCoralMode()) {
    } else {
      Elevator.getInstance().setTarget(TargetType.ALGAE_PROCESSOR);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (lastMode != Claw.getInstance().getGamePieceMode()) {
      if (Claw.getInstance().isCoralMode()) {
      } else {
        Elevator.getInstance().setTarget(TargetType.ALGAE_PROCESSOR);
      }
    }

    switch (level_) {
      case L1:
        Elevator.getInstance()
            .setTarget(
                Claw.getInstance().isCoralMode() ? TargetType.L1 : TargetType.ALGAE_PROCESSOR);
        break;

      case L2:
        Elevator.getInstance()
            .setTarget(Claw.getInstance().isCoralMode() ? TargetType.L2 : TargetType.ALGAE_LOW);
        break;

      case L3:
        Elevator.getInstance()
            .setTarget(Claw.getInstance().isCoralMode() ? TargetType.L3 : TargetType.ALGAE_HIGH);
        break;

      case L4:
        Elevator.getInstance()
            .setTarget(Claw.getInstance().isCoralMode() ? TargetType.L4 : TargetType.BARGE);
        break;
      default:
        // hello :)
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
