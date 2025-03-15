package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.mw_lib.auto.Auto;
import frc.robot.AutoManager;
import frc.robot.commands.AutoCoralReefScore;
import frc.robot.commands.CoralLoad;
import frc.robot.subsystems.GameStateManager;
import frc.robot.subsystems.GameStateManager.Column;
import frc.robot.subsystems.GameStateManager.ReefScoringTarget;

public class J4_LeftStation_L4_LeftStation_K4 extends Auto {

  public J4_LeftStation_L4_LeftStation_K4() {
    this.addCommands(
        // Score game Piece 1
        GameStateManager.setScoringCommand(Column.RIGHT, ReefScoringTarget.L4),
        AutoManager.getInstance().getAutoFactory().trajectoryCmd("Blue Mid to IJ"),
        new AutoCoralReefScore(),

        // Get game piece 2
        AutoManager.getInstance()
            .getAutoFactory()
            .trajectoryCmd("IJ to Left Station")
            .alongWith(new CoralLoad().beforeStarting(new WaitCommand(0.5))),

        // Score game piece 2
        GameStateManager.setScoringCommand(Column.LEFT, ReefScoringTarget.L4),
        AutoManager.getInstance().getAutoFactory().trajectoryCmd("Left Station to KL"),
        new AutoCoralReefScore(),

        // Get game piece 3
        AutoManager.getInstance()
            .getAutoFactory()
            .trajectoryCmd("KL to Left Station")
            .alongWith(new CoralLoad().beforeStarting(new WaitCommand(0.5))),

        // Score game piece 3
        GameStateManager.setScoringCommand(Column.RIGHT, ReefScoringTarget.L4),
        AutoManager.getInstance().getAutoFactory().trajectoryCmd("Left Station to KL"),
        new AutoCoralReefScore());

    // Setup Path List
    trajectory_list_.add(choreo.Choreo.loadTrajectory("Blue Mid to IJ").get().getPoses());
    trajectory_list_.add(choreo.Choreo.loadTrajectory("IJ to Left Station").get().getPoses());
    trajectory_list_.add(choreo.Choreo.loadTrajectory("Left Station to KL").get().getPoses());
    trajectory_list_.add(choreo.Choreo.loadTrajectory("KL to Left Station").get().getPoses());
    trajectory_list_.add(choreo.Choreo.loadTrajectory("Left Station to KL").get().getPoses());
  }
}
