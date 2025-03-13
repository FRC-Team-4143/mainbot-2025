package frc.robot.autos;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.AutoManager;
import frc.robot.commands.AutoCoralReefScore;
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
    AutoTrajectory LeftStation_to_KL_2 = routine.trajectory("Left Station to KL");

    // Drive to IJ Face
    routine.active().onTrue(Mid_to_IJ.cmd());
    // Score on J4 and Go to Left Station
    Mid_to_IJ.done()
        .onTrue(
            Commands.sequence(
                Commands.runOnce(
                    () ->
                        GameStateManager.getInstance()
                            .setScoringObj(Column.RIGHT, ReefScoringTarget.L4, false)),
                new AutoCoralReefScore(),
                IJ_to_LeftStation.cmd()));
    // Drive to KL once Coral is Loaded
    IJ_to_LeftStation.done().onTrue(LeftStation_to_KL.cmd());
    // Score on K4 and Go to Left Station
    LeftStation_to_KL.done()
        .onTrue(
            Commands.sequence(
                Commands.runOnce(
                    () ->
                        GameStateManager.getInstance()
                            .setScoringObj(Column.LEFT, ReefScoringTarget.L4, false)),
                new AutoCoralReefScore(),
                KL_to_LeftStation.cmd()));
    // Drive to KL once Coral is Loaded
    KL_to_LeftStation.done().onTrue(LeftStation_to_KL_2.cmd());
    // Score on L4
    LeftStation_to_KL_2.done()
        .onTrue(
            Commands.sequence(
                Commands.runOnce(
                    () ->
                        GameStateManager.getInstance()
                            .setScoringObj(Column.RIGHT, ReefScoringTarget.L4, false)),
                new AutoCoralReefScore()));

    return routine;
  }
}
