package frc.robot.autos;

import frc.mw_lib.auto.Auto;
import frc.robot.commands.AutoCoralReefScore;
import frc.robot.commands.CoralTractorBeam;
import frc.robot.commands.IntakeHandoff;
import frc.robot.subsystems.CoralDetector;
import frc.robot.subsystems.GameStateManager;
import frc.robot.subsystems.GameStateManager.Column;
import frc.robot.subsystems.GameStateManager.ReefScoringTarget;

public class Left_Ground_Test extends Auto {

  public Left_Ground_Test() {
    // Register the paths first
    this.loadTrajectory("Left Start to IJ");
    this.loadTrajectory("IJ to Left Ground");
    this.loadTrajectory("Left Ground to KL");
    this.loadTrajectory("KL to Left Ground");
    this.loadTrajectory("Left Ground to KL");

    this.addCommands(
        // Score game Piece 1
        this.getTrajectoryCmd("Left Start to IJ"),
        GameStateManager.setScoringCommand(Column.RIGHT, ReefScoringTarget.L4),
        new AutoCoralReefScore(),

        // Get game Piece 2
        this.getTrajectoryCmd("IJ to Left Ground")
            .until(CoralDetector.getInstance()::isValid)
            .raceWith(new IntakeHandoff()),
        new CoralTractorBeam(),

        // Score game Piece 2
        this.getTrajectoryCmd("Left Ground to KL").alongWith(new IntakeHandoff()),
        GameStateManager.setScoringCommand(Column.LEFT, ReefScoringTarget.L4),
        new AutoCoralReefScore(),

        // Get game Piece 3
        this.getTrajectoryCmd("KL to Left Ground")
            .until(CoralDetector.getInstance()::isValid)
            .raceWith(new IntakeHandoff()),
        new CoralTractorBeam(),

        // Score game Piece 3
        this.getTrajectoryCmd("Left Ground to KL").alongWith(new IntakeHandoff()),
        GameStateManager.setScoringCommand(Column.RIGHT, ReefScoringTarget.L4),
        new AutoCoralReefScore());
  }
}
