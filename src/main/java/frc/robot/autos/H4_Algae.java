package frc.robot.autos;

import frc.mw_lib.auto.Auto;
import frc.robot.commands.AutoAlgaeReefPickup;
import frc.robot.commands.AutoCoralReefScore;
import frc.robot.subsystems.GameStateManager;
import frc.robot.subsystems.GameStateManager.Column;
import frc.robot.subsystems.GameStateManager.ReefScoringTarget;

public class H4_Algae extends Auto {

  public H4_Algae() {
    // Register the paths first
    this.loadTrajectory("Mid Start to GH");
    this.loadTrajectory("GH to GH");

    this.addCommands(
        // Score game Piece 1
        GameStateManager.setScoringCommand(Column.RIGHT, ReefScoringTarget.L4),
        // this.getTrajectoryCmd("Mid Start to GH"),
        new AutoCoralReefScore(),

        // Get game piece 2
        this.getTrajectoryCmd("GH to GH Forward"),
        new AutoAlgaeReefPickup(),

        // Go to barge
        this.getTrajectoryCmd("GH To Barge"));
  }
}
