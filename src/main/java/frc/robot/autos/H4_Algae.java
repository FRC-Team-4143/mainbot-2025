package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.ElevatorTargets.TargetType;
import frc.mw_lib.auto.Auto;
import frc.robot.commands.AutoAlgaeReefPickup;
import frc.robot.commands.AutoCoralReefScore;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.GameStateManager;
import frc.robot.subsystems.Claw.ClawMode;
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
        // this.getTrajectoryCmd("Mid Start to GH"),
        new AutoCoralReefScore(),

        // Get game piece 2
        this.getTrajectoryCmd("GH to Mid"),
        new AutoAlgaeReefPickup(),

        // Go to barge
        this.getTrajectoryCmd("GH To Barge").alongWith(
        new WaitCommand(0.25)).andThen(
        Commands.runOnce(() -> Elevator.getInstance().setTarget(TargetType.BARGE))),

        // score piece 2
        Commands.run(() -> Claw.getInstance().setClawMode(ClawMode.SHOOT)).withTimeout(1),

        // get game piece 3
        this.getTrajectoryCmd("Barge to IJ").alongWith(
        new WaitCommand(0.25)).andThen(
        Commands.runOnce(() -> Elevator.getInstance().setTarget(TargetType.ALGAE_HIGH))),
        new AutoAlgaeReefPickup(),

        //Go to barge
        this.getTrajectoryCmd("IJ to Barge").alongWith(
          new WaitCommand(0.25)).andThen(
          Commands.runOnce(() -> Elevator.getInstance().setTarget(TargetType.BARGE))),

        // score piece 3
        Commands.run(() -> Claw.getInstance().setClawMode(ClawMode.SHOOT)).withTimeout(1),

        this.getTrajectoryCmd("Barge to IJ").alongWith(Commands.runOnce(() -> Elevator.getInstance().setTarget(TargetType.ALGAE_STOW)))
    );
  }
}
