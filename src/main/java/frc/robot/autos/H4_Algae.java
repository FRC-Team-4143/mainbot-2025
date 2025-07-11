package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.ElevatorTargets.TargetType;
import frc.mw_lib.auto.Auto;
import frc.robot.commands.AutoAlgaeReefPickup;
import frc.robot.commands.AutoCoralReefScore;
import frc.robot.commands.AutoScoreBarge;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.GameStateManager;
import frc.robot.subsystems.GameStateManager.Column;
import frc.robot.subsystems.GameStateManager.ReefScoringTarget;

public class H4_Algae extends Auto {

  public H4_Algae() {
    // Register the paths first
    this.loadTrajectory("GH to Mid");
    this.loadTrajectory("GH To Barge");
    this.loadTrajectory("Barge to IJ");
    this.loadTrajectory("IJ to Barge");

    this.addCommands(
        // Score game Piece 1
        GameStateManager.setScoringCommand(Column.RIGHT, ReefScoringTarget.L4),
        new AutoCoralReefScore(),

        // Get game piece 2
        this.getTrajectoryCmd("GH to Mid"),
        new AutoAlgaeReefPickup(),

        // Go to barge
        this.getTrajectoryCmd("GH To Barge")
            .raceWith(
                new WaitCommand(0.1)
                    .andThen(
                        Commands.run(
                            () -> Elevator.getInstance().setTarget(TargetType.BARGE),
                            Elevator.getInstance()))),

        // score piece 2
        new AutoScoreBarge().withTimeout(2),

        // get game piece 3
        this.getTrajectoryCmd("Barge to IJ"),
        new AutoAlgaeReefPickup(),

        // Go to barge
        this.getTrajectoryCmd("IJ to Barge")
            .raceWith(
                new WaitCommand(0.1)
                    .andThen(
                        Commands.run(
                            () -> Elevator.getInstance().setTarget(TargetType.BARGE),
                            Elevator.getInstance()))),

        // score piece 3
        new AutoScoreBarge().withTimeout(2),
        this.getTrajectoryCmd("Barge to IJ")
            .alongWith(
                Commands.runOnce(() -> Elevator.getInstance().setTarget(TargetType.ALGAE_STOW))));
  }
}
