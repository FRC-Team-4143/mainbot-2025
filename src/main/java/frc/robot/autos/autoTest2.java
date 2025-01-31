package frc.robot.autos;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.AutoManager;
import frc.robot.commands.Score;

public class autoTest2 {

  public static AutoRoutine getAutoRoutine() {

    AutoFactory autoFactory = AutoManager.getInstance().getAutoFactory();
    AutoRoutine routine = autoFactory.newRoutine("autoTest2");
    AutoTrajectory Score1Back = routine.trajectory("Score1Back");

    routine.active().onTrue(Commands.sequence(Score1Back.cmd())); // Score1Back.resetOdometry()
    Score1Back.done().onTrue(new Score().withTimeout(2));

    return routine;
  }
}
