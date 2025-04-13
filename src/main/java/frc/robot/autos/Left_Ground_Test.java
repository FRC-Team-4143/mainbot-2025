package frc.robot.autos;

import frc.mw_lib.auto.Auto;
import frc.robot.commands.AutoAlgaeReefPickup;
import frc.robot.commands.AutoCoralReefScore;
import frc.robot.commands.CoralTractorBeam;
import frc.robot.subsystems.CoralDetector;
import frc.robot.subsystems.GameStateManager;
import frc.robot.subsystems.Pickup;
import frc.robot.subsystems.GameStateManager.Column;
import frc.robot.subsystems.GameStateManager.ReefScoringTarget;

public class Left_Ground_Test extends Auto {

  public Left_Ground_Test() {
    // Register the paths first
    this.loadTrajectory("Left Start to IJ");
    this.loadTrajectory("IJ to Left Ground");
    this.loadTrajectory("Left Ground to KL");

    this.addCommands(
        // Score game Piece 1
        this.getTrajectoryCmd("Left Start to IJ"),
        GameStateManager.setScoringCommand(Column.LEFT, ReefScoringTarget.L4),
        new AutoCoralReefScore(),

        // Get game piece 2
        this.getTrajectoryCmd("IJ to Left Ground").until(CoralDetector.getInstance()::isValid),
        new CoralTractorBeam(),
        
        // Score game Piece 2
        this.getTrajectoryCmd("Left Ground to KL"),
        GameStateManager.setScoringCommand(Column.LEFT, ReefScoringTarget.L4),
        new AutoCoralReefScore()
    );     
  }
}
