package frc.robot.autos;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.AutoManager;
import frc.robot.commands.AutoCoralReefScore;
import frc.robot.commands.CoralLoad;
import frc.robot.subsystems.GameStateManager;
import frc.robot.subsystems.GameStateManager.Column;
import frc.robot.subsystems.GameStateManager.ReefScoringTarget;

public class J4_LeftStation_L4_LeftStation_K4 {

  public static AutoRoutine getAutoRoutine() {

    AutoFactory autoFactory = AutoManager.getInstance().getAutoFactory();
    AutoRoutine routine = autoFactory.newRoutine("J4_LeftStation_L4_LeftStation_K4");

    AutoTrajectory Mid_to_IJ = routine.trajectory("Blue Mid to IJ");
    AutoTrajectory IJ_to_LeftStation = routine.trajectory("IJ to Left Station");
    AutoTrajectory KL_to_LeftStation = routine.trajectory("KL to Left Station");
    AutoTrajectory LeftStation_to_KL = routine.trajectory("Left Station to KL");

    Command seq_cmd =
        Commands.sequence(
            // Score game Piece 1
            GameStateManager.setScoringCommand(Column.RIGHT, ReefScoringTarget.L4),
            Mid_to_IJ.cmd(),
            new AutoCoralReefScore(),

            // Get game piece 2
            IJ_to_LeftStation.cmd().alongWith(new CoralLoad().beforeStarting(new WaitCommand(0.5))),

            // Score game piece 2
            GameStateManager.setScoringCommand(Column.LEFT, ReefScoringTarget.L4),
            LeftStation_to_KL.cmd(),
            new AutoCoralReefScore(),

            // Get game piece 3
            KL_to_LeftStation.cmd().alongWith(new CoralLoad().beforeStarting(new WaitCommand(0.5))),

            // Score game piece 3
            GameStateManager.setScoringCommand(Column.RIGHT, ReefScoringTarget.L4),
            LeftStation_to_KL.cmd(),
            new AutoCoralReefScore());

    routine.active().onTrue(seq_cmd);

    return routine;
  }
}
