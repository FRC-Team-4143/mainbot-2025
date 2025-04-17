package frc.robot.autos;

import frc.mw_lib.auto.Auto;
import frc.robot.commands.AutoCoralReefScore;
import frc.robot.commands.CoralTractorBeam;
import frc.robot.commands.IntakeHandoff;
import frc.robot.subsystems.CoralDetector;
import frc.robot.subsystems.GameStateManager;
import frc.robot.subsystems.GameStateManager.Column;
import frc.robot.subsystems.GameStateManager.ReefScoringTarget;

public class Right_3_Piece extends Auto {

  public Right_3_Piece() {
    // Register the paths first
    this.loadTrajectory("Right Start to EF");
    this.loadTrajectory("EF to Right Ground");
    this.loadTrajectory("Right Ground to CD");
    this.loadTrajectory("CD to Right Ground");
    this.loadTrajectory("Right Ground to CD");

    this.addCommands(
        // Score game Piece 1
        this.getTrajectoryCmd("Right Start to EF"),
        GameStateManager.setScoringCommand(Column.RIGHT, ReefScoringTarget.L4),
        new AutoCoralReefScore(),

        // Get game Piece 2
        this.getTrajectoryCmd("EF to Right Ground")
            .until(CoralDetector.getInstance()::isValid)
            .raceWith(new IntakeHandoff()),
        new CoralTractorBeam(),

        // Score game Piece 2
        this.getTrajectoryCmd("Right Ground to CD").alongWith(new IntakeHandoff()),
        GameStateManager.setScoringCommand(Column.LEFT, ReefScoringTarget.L4),
        new AutoCoralReefScore(),

        // Get game Piece 3
        this.getTrajectoryCmd("CD to Right Ground")
            .until(CoralDetector.getInstance()::isValid)
            .raceWith(new IntakeHandoff()),
        new CoralTractorBeam(),

        // Score game Piece 3
        this.getTrajectoryCmd("Right Ground to CD").alongWith(new IntakeHandoff()),
        GameStateManager.setScoringCommand(Column.RIGHT, ReefScoringTarget.L4),
        new AutoCoralReefScore());
  }
}
