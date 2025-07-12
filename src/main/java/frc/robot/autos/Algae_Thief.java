package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.ElevatorTargets.TargetType;
import frc.mw_lib.auto.Auto;
import frc.robot.commands.AutoAlgaeReefPickup;
import frc.robot.commands.AutoCoralReefScore;
import frc.robot.commands.AutoScoreBargeSteal;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Claw.GamePiece;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.GameStateManager;
import frc.robot.subsystems.GameStateManager.Column;
import frc.robot.subsystems.GameStateManager.ReefScoringTarget;

public class Algae_Thief extends Auto {

  public Algae_Thief() {
    // Register the paths first
    this.loadTrajectory("GH to OP-EF");
    this.loadTrajectory("OP-EF to Far Barge");
    this.loadTrajectory("Far Barge to OP-GH");
    this.loadTrajectory("OP-GH to Far Barge");
    this.loadTrajectory("Far Barge to Safe");

    this.addCommands(
        // Score game Piece 1
        GameStateManager.setScoringCommand(Column.RIGHT, ReefScoringTarget.L4),
        new AutoCoralReefScore(),

        // Get game piece 2
        this.getTrajectoryCmd("GH to OP-EF")
            .raceWith(
                new WaitCommand(2.5)
                    .andThen(Commands.run(() -> Claw.getInstance().setGamePiece(GamePiece.ALGAE)))),
        new AutoAlgaeReefPickup(),
        // Go to barge
        this.getTrajectoryCmd("OP-EF to Far Barge")
            .raceWith(
                new WaitCommand(0.1)
                    .andThen(
                        Commands.run(
                            () -> Elevator.getInstance().setTarget(TargetType.BARGE),
                            Elevator.getInstance()))),

        // score piece 2
        new AutoScoreBargeSteal().withTimeout(2),

        // get game piece 3
        this.getTrajectoryCmd("Far Barge to OP-GH"),
        new AutoAlgaeReefPickup(),

        // Go to barge
        this.getTrajectoryCmd("OP-GH to Far Barge")
            .raceWith(
                new WaitCommand(0.1)
                    .andThen(
                        Commands.run(
                            () -> Elevator.getInstance().setTarget(TargetType.BARGE),
                            Elevator.getInstance()))),

        // score piece 3
        new AutoScoreBargeSteal().withTimeout(2),

        // back off
        this.getTrajectoryCmd("Far Barge to Safe")
            .alongWith(
                Commands.runOnce(() -> Elevator.getInstance().setTarget(TargetType.ALGAE_STOW))));
  }
}
