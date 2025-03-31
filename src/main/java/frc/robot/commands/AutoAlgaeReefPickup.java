// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.ElevatorTargets.TargetType;
import frc.lib.FieldRegions;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Claw.ClawMode;
import frc.robot.subsystems.Claw.GamePiece;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.SpeedLimit;
import frc.robot.subsystems.GameStateManager;
import frc.robot.subsystems.GameStateManager.Column;
import frc.robot.subsystems.GameStateManager.ReefScoringTarget;
import frc.robot.subsystems.GameStateManager.RobotState;
import frc.robot.subsystems.PoseEstimator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAlgaeReefPickup extends Command {
  /** Creates a new AutoAlgaeReefPickup. */
  public AutoAlgaeReefPickup() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Elevator.getInstance());
    setName(this.getClass().getSimpleName());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Elevator.getInstance().setSpeedLimit(SpeedLimit.CORAL);
    GameStateManager.getInstance().setScoringTarget(ReefScoringTarget.ALGAE, false);
    GameStateManager.getInstance().setScoringColum(Column.ALGAE, false);
    Claw.getInstance().setGamePiece(GamePiece.ALGAE);
    Claw.getInstance().setClawMode(ClawMode.LOAD);
    GameStateManager.getInstance().setRobotState(RobotState.TARGET_ACQUISITION);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    GameStateManager.getInstance().setRobotState(RobotState.END);
    Claw.getInstance().setClawMode(ClawMode.IDLE);
    for (int i = 0; i < FieldRegions.REEF_REGIONS.length; i++) {
      if (FieldRegions.REEF_REGIONS[i].contains(PoseEstimator.getInstance().getRobotPose())) {
        if ((i % 2) == 0) {
          Elevator.getInstance().setTarget(TargetType.ALGAE_HIGH);
          break;
        } else {
          Elevator.getInstance().setTarget(TargetType.ALGAE_LOW);
          break;
        }
      }
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Claw.getInstance().hasAlgae();
  }
}
