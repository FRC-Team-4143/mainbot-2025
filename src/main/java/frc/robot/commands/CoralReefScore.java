// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.lib.FieldRegions;
import frc.mw_lib.command.LazyCommand;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Claw.ClawMode;
import frc.robot.subsystems.Claw.GamePiece;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.GameStateManager;
import frc.robot.subsystems.GameStateManager.GameStateTarget;
import frc.robot.subsystems.GameStateManager.RobotState;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.ReefObserver;
import java.util.Optional;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralReefScore extends LazyCommand {
  /** Creates a new CoralReefScore. */
  public CoralReefScore() {
    super(2);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Elevator.getInstance());
    setName(this.getClass().getSimpleName());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.timerReset();
    GameStateManager.getInstance().setRobotState(RobotState.TARGET_ACQUISITION);
    Optional<GameStateTarget> target = ReefObserver.getInstance().findNextTarget();
    if (target.isPresent()) {
      GameStateManager.getInstance().setScoringObj(target.get(), false);
    } else {
      end(false);
    }
    Claw.getInstance().setGamePiece(GamePiece.CORAL);
    Claw.getInstance().setClawMode(ClawMode.IDLE);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    GameStateManager.getInstance().setRobotState(RobotState.END);
    Claw.getInstance().setClawMode(ClawMode.IDLE);
  }

  // Returns true when the command should end.
  @Override
  public boolean isConditionMet() {
    return !(GameStateManager.getInstance().isRunning()
        && FieldRegions.REEF_EXIT_REGION.contains(PoseEstimator.getInstance().getRobotPose()));
  }
}
