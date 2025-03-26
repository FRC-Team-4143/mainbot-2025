package frc.robot.autos;

import frc.mw_lib.auto.Auto;
import frc.robot.commands.AutoCoralReefScore;
import frc.robot.commands.IntakeHandoff;
import frc.robot.subsystems.GameStateManager;
import frc.robot.subsystems.GameStateManager.Column;
import frc.robot.subsystems.GameStateManager.ReefScoringTarget;

public class Bump_J4_LeftStationGround_L4_LeftStationGround_D4 extends Auto {

  public Bump_J4_LeftStationGround_L4_LeftStationGround_D4() {
    // Register Trajectories first
    this.loadTrajectory("Bump Left Start to IJ");
    this.loadTrajectory("IJ to KL Station");
    this.loadTrajectory("KL to KL Station");

    this.addCommands(
        // Score game Piece 1
        GameStateManager.setScoringCommand(Column.RIGHT, ReefScoringTarget.L4),
        this.getTrajectoryCmd("Bump Left Start to IJ"),
        new AutoCoralReefScore(),

        // Get game piece 2
        new IntakeHandoff(),
        this.getTrajectoryCmd("IJ to KL Station"),

        // Score game piece 2
        GameStateManager.setScoringCommand(Column.LEFT, ReefScoringTarget.L4),
        new AutoCoralReefScore(),

        // Get game piece 3
        new IntakeHandoff(),
        this.getTrajectoryCmd("KL to KL Station"),

        // Score game piece 3
        GameStateManager.setScoringCommand(Column.RIGHT, ReefScoringTarget.L4),
        new AutoCoralReefScore());
  }
}
