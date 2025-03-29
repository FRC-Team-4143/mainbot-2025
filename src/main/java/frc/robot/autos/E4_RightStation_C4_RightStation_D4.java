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

public class E4_RightStation_C4_RightStation_D4 extends Auto {

  public E4_RightStation_C4_RightStation_D4() {
    // Register Trajectories first
    this.loadTrajectory("Right Start to EF");
    this.loadTrajectory("EF to Right Station");
    this.loadTrajectory("Right Station to CD");
    this.loadTrajectory("CD to Right Station");
    this.loadTrajectory("Right Station to CD");

    this.addCommands(
        // Score game Piece 1
        GameStateManager.setScoringCommand(Column.RIGHT, ReefScoringTarget.L4),
        this.getTrajectoryCmd("Right Start to EF"),
        Commands.parallel(new AutoCoralReefScore(), new SetIntake().withTimeout(1.0)),

        // Get game piece 2
        this.getTrajectoryCmd("EF to Right Station")
            .alongWith(new CoralLoad().beforeStarting(new WaitCommand(0.5))),

        // Score game piece 2
        GameStateManager.setScoringCommand(Column.LEFT, ReefScoringTarget.L4),
        this.getTrajectoryCmd("Right Station to CD"),
        new AutoCoralReefScore(),

        // Get game piece 3
        this.getTrajectoryCmd("CD to Right Station")
            .alongWith(new CoralLoad().beforeStarting(new WaitCommand(0.5))),

        // Score game piece 3
        GameStateManager.setScoringCommand(Column.RIGHT, ReefScoringTarget.L4),
        this.getTrajectoryCmd("Right Station to CD"),
        new AutoCoralReefScore());
  }
}
