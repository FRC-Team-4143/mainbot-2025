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
    AutoTrajectory score1 = routine.trajectory("Score1");

    routine.active().onTrue(Commands.sequence(score1.resetOdometry(), score1.cmd()));

    score1.done().onTrue(new Score().withTimeout(2));

    return routine;
  }
}
