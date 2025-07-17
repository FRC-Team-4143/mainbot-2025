package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.ElevatorTargets;
import frc.mw_lib.auto.Auto;
import frc.robot.commands.AutoCoralReefScore;
import frc.robot.commands.CoralTractorBeam;
import frc.robot.commands.IntakeHandoff;
import frc.robot.subsystems.CoralDetector;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.GameStateManager;
import frc.robot.subsystems.GameStateManager.Column;
import frc.robot.subsystems.GameStateManager.ReefScoringTarget;

public class Left_3_Piece extends Auto {

  public Left_3_Piece() {
    // Register the paths first
    this.loadTrajectory("Left Start to IJ");
    this.loadTrajectory("IJ to Left Ground");
    this.loadTrajectory("Left Ground to KL");
    this.loadTrajectory("KL to Left Ground");
    this.loadTrajectory("Left Ground to KL");

    this.addCommands(
        // Score game Piece 1
        GameStateManager.setScoringCommand(Column.RIGHT, ReefScoringTarget.L4),
        this.getTrajectoryCmd("Left Start to IJ")
            .raceWith(
                new WaitCommand(0.0)
                    .andThen(
                        Commands.run(
                            () -> Elevator.getInstance().setTarget(ElevatorTargets.TargetType.L4),
                            Elevator.getInstance()))),
        new AutoCoralReefScore(),

        // Get game Piece 2
        this.getTrajectoryCmd("IJ to Left Ground").until(CoralDetector.getInstance()::isValid),
        new CoralTractorBeam().withTimeout(5),

        // Score game Piece 2
        GameStateManager.setScoringCommand(Column.LEFT, ReefScoringTarget.L4),
        this.getTrajectoryCmd("Left Ground to KL")
            .raceWith(
                new IntakeHandoff()
                    .andThen(
                        Commands.run(
                            () -> Elevator.getInstance().setTarget(ElevatorTargets.TargetType.L4),
                            Elevator.getInstance()))),
        new AutoCoralReefScore(),

        // Get game Piece 3
        this.getTrajectoryCmd("KL to Left Ground").until(CoralDetector.getInstance()::isValid),
        new CoralTractorBeam().withTimeout(5),

        // Score game Piece 3
        GameStateManager.setScoringCommand(Column.RIGHT, ReefScoringTarget.L4),
        this.getTrajectoryCmd("Left Ground to KL")
            .raceWith(
                new IntakeHandoff()
                    .andThen(
                        Commands.run(
                            () -> Elevator.getInstance().setTarget(ElevatorTargets.TargetType.L4),
                            Elevator.getInstance()))),
        new AutoCoralReefScore(),
        // Get game piece 4
        this.getTrajectoryCmd("KL to Left Ground").until(CoralDetector.getInstance()::isValid),
        new CoralTractorBeam().withTimeout(5));
  }
}
