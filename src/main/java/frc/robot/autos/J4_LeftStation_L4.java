package frc.robot.autos;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.AutoManager;
import frc.robot.commands.AutoCoralReefScore;
import frc.robot.commands.CoralLoad;
import frc.robot.subsystems.GameStateManager.Column;
import frc.robot.subsystems.GameStateManager.ReefScoringTarget;

public class J4_LeftStation_L4 {

  public static AutoRoutine getAutoRoutine() {

    AutoFactory autoFactory = AutoManager.getInstance().getAutoFactory();
    AutoRoutine routine = autoFactory.newRoutine("J4_LeftStation_L4");

    AutoTrajectory Mid_to_IJ = routine.trajectory("Blue Mid to IJ");
    AutoTrajectory IJ_to_LeftStation = routine.trajectory("IJ to Left Station");
    AutoTrajectory LeftStation_to_KL = routine.trajectory("Left Station to KL");

    routine.active().onTrue(Mid_to_IJ.cmd());
    Mid_to_IJ.inactive()
        .onTrue(
            Commands.sequence(
                new AutoCoralReefScore(ReefScoringTarget.L4, Column.RIGHT),
                IJ_to_LeftStation.cmd()));
    IJ_to_LeftStation.done()
        .onTrue(Commands.sequence(new CoralLoad().withTimeout(5), LeftStation_to_KL.cmd()));
    LeftStation_to_KL.inactive().onTrue(new AutoCoralReefScore(ReefScoringTarget.L4, Column.LEFT));

    return routine;
  }
}
