package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.mw_lib.auto.Auto;
import frc.robot.commands.AutoCoralReefScore;
import frc.robot.commands.CoralLoad;
import frc.robot.commands.SetIntake;
import frc.robot.subsystems.GameStateManager;
import frc.robot.subsystems.GameStateManager.Column;
import frc.robot.subsystems.GameStateManager.ReefScoringTarget;

public class J4_LeftStation_L4 extends Auto {

  public J4_LeftStation_L4() {
    // Register the paths first
    this.loadTrajectory("Left Start to IJ");
    this.loadTrajectory("IJ to Left Station");
    this.loadTrajectory("Left Station to KL");

    this.addCommands(
        // Score game Piece 1
        GameStateManager.setScoringCommand(Column.RIGHT, ReefScoringTarget.L4),
        this.getTrajectoryCmd("Left Start to IJ"),
        Commands.parallel(new AutoCoralReefScore(), new SetIntake().withTimeout(1.0)),

        // Get game piece 2
        this.getTrajectoryCmd("IJ to Left Station")
            .alongWith(new CoralLoad().beforeStarting(new WaitCommand(0.5))),

        // Score game piece 2
        GameStateManager.setScoringCommand(Column.LEFT, ReefScoringTarget.L4),
        this.getTrajectoryCmd("Left Station to KL"),
        new AutoCoralReefScore());
  }
}
