// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Claw.ClawMode;
import frc.robot.subsystems.Claw.GamePiece;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.GameStateManager;
import frc.robot.subsystems.GameStateManager.RobotState;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralReefScore extends Command {
  /** Creates a new CoralReefScore. */
  public CoralReefScore() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Elevator.getInstance());
    setName(this.getClass().getSimpleName());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    GameStateManager.getInstance().setRobotState(RobotState.TARGET_ACQUISITION);
    GameStateManager.getInstance()
        .setScoringTarget(GameStateManager.getInstance().getSavedScoringTarget(), true);
    GameStateManager.getInstance()
        .setScoringColum(GameStateManager.getInstance().getSavedScoringColum(), true);
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
  public boolean isFinished() {
    return false;
  }
}
