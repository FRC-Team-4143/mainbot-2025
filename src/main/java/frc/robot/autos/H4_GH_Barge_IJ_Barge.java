package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.mw_lib.auto.Auto;
import frc.robot.commands.AutoAlgaeReefPickup;
import frc.robot.commands.AutoCoralReefScore;
import frc.robot.commands.AutoScoreBarge;
import frc.robot.subsystems.GameStateManager;
import frc.robot.subsystems.GameStateManager.Column;
import frc.robot.subsystems.GameStateManager.ReefScoringTarget;

public class H4_GH_Barge_IJ_Barge extends Auto {

  public H4_GH_Barge_IJ_Barge() {
    // this.setName(getClass().getSimpleName());

    // Register the paths first
    this.loadTrajectory("Mid Start to GH");
    this.loadTrajectory("GH to GH");
    this.loadTrajectory("GH to Barge");
    this.loadTrajectory("Barge to IJ");
    this.loadTrajectory("IJ to Barge");

    this.addCommands(
        // Score game Piece 1
        GameStateManager.setScoringCommand(Column.RIGHT, ReefScoringTarget.L4),
        this.getTrajectoryCmd("Mid Start to GH"),
        new AutoCoralReefScore(),

        // Get game piece 2
        this.getTrajectoryCmd("GH to GH"),
        new AutoAlgaeReefPickup(),

        // Score game piece 2
        this.getTrajectoryCmd("GH to Barge"),
        new AutoScoreBarge().withTimeout(3),

        // Get game piece 3
        this.getTrajectoryCmd("Barge to IJ").beforeStarting((new WaitCommand(1))),
        new AutoAlgaeReefPickup(),

        // Score game piece 3
        this.getTrajectoryCmd("IJ to Barge"),
        new AutoScoreBarge().withTimeout(3));
  }
}
