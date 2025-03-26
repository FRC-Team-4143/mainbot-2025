package frc.robot.autos;

import frc.mw_lib.auto.Auto;
import frc.robot.commands.AutoCoralReefScore;
import frc.robot.commands.IntakeHandoff;
import frc.robot.subsystems.GameStateManager;
import frc.robot.subsystems.GameStateManager.Column;
import frc.robot.subsystems.GameStateManager.ReefScoringTarget;

public class Bump_E4_RightStationGround_C4_RightStationGround_D4 extends Auto {

  public Bump_E4_RightStationGround_C4_RightStationGround_D4() {
    // Register Trajectories first
    this.loadTrajectory("Bump Right Start to EF");
    this.loadTrajectory("EF to CD Station");
    this.loadTrajectory("CD to CD Station");

    this.addCommands(
        // Score game Piece 1
        GameStateManager.setScoringCommand(Column.RIGHT, ReefScoringTarget.L4),
        this.getTrajectoryCmd("Bump Right Start to EF"),
        new AutoCoralReefScore(),

        // Get game piece 2
        new IntakeHandoff(),
        this.getTrajectoryCmd("EF to CD Station"),

        // Score game piece 2
        GameStateManager.setScoringCommand(Column.LEFT, ReefScoringTarget.L4),
        new AutoCoralReefScore(),

        // Get game piece 3
        new IntakeHandoff(),
        this.getTrajectoryCmd("CD to CD Station"),

        // Score game piece 3
        GameStateManager.setScoringCommand(Column.RIGHT, ReefScoringTarget.L4),
        new AutoCoralReefScore());
  }
}
