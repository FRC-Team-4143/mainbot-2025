package frc.robot.autos;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.AutoManager;
import frc.robot.commands.Feed;
import frc.robot.commands.Score;

public class autoPresentation {

  public static AutoRoutine getAutoRoutine() {

    AutoFactory autoFactory = AutoManager.getInstance().getAutoFactory();
    AutoRoutine routine = autoFactory.newRoutine("presentation");
    AutoTrajectory SourceToA = routine.trajectory("Source To A");
    AutoTrajectory SourceToB = routine.trajectory("Source To B");
    AutoTrajectory SourceToC = routine.trajectory("Source To C");
    AutoTrajectory SourceToD = routine.trajectory("Source To D");
    AutoTrajectory SourceToE = routine.trajectory("Source To E");
    AutoTrajectory SourceToF = routine.trajectory("Source To F");
    AutoTrajectory SourceToG = routine.trajectory("Source To G");
    AutoTrajectory SourceToH = routine.trajectory("Source To H");
    AutoTrajectory SourceToI = routine.trajectory("Source To I");
    AutoTrajectory SourceToJ = routine.trajectory("Source To J");
    AutoTrajectory SourceToK = routine.trajectory("Source To K");
    AutoTrajectory SourceToL = routine.trajectory("Source To L");
    AutoTrajectory SourceBackAB = routine.trajectory("Source Back AB");
    AutoTrajectory SourceBackCD = routine.trajectory("Source Back CD");
    AutoTrajectory SourceBackEF = routine.trajectory("Source Back EF");
    AutoTrajectory SourceBackGH = routine.trajectory("Source Back GH");
    AutoTrajectory SourceBackIJ = routine.trajectory("Source Back IJ");
    AutoTrajectory SourceBackKL = routine.trajectory("Source Back KL");

    routine.active().onTrue(Commands.sequence(SourceToA.resetOdometry(), SourceToA.cmd()));
    SourceToA.done().onTrue(Commands.sequence(new Score().withTimeout(1), SourceBackAB.cmd()));
    SourceBackAB.done().onTrue(Commands.sequence(new Feed(), SourceToB.cmd()));
    SourceToB.done().onTrue((Commands.sequence(new Score().withTimeout(1), SourceBackAB.cmd())));
    SourceBackAB.done().onTrue((Commands.sequence(new Feed(), SourceToC.cmd())));
    SourceToC.done().onTrue(Commands.sequence(new Score().withTimeout(1), SourceBackCD.cmd()));
    SourceBackCD.done().onTrue((Commands.sequence(new Feed(), SourceToD.cmd())));
    SourceToD.done().onTrue(Commands.sequence(new Score().withTimeout(1), SourceBackCD.cmd()));
    SourceBackCD.done().onTrue((Commands.sequence(new Feed(), SourceToE.cmd())));
    SourceToE.done().onTrue(Commands.sequence(new Score().withTimeout(1), SourceBackEF.cmd()));
    SourceBackEF.done().onTrue((Commands.sequence(new Feed(), SourceToF.cmd())));
    SourceToF.done().onTrue(Commands.sequence(new Score().withTimeout(1), SourceBackEF.cmd()));
    SourceBackEF.done().onTrue((Commands.sequence(new Feed(), SourceToG.cmd())));
    SourceToG.done().onTrue(Commands.sequence(new Score().withTimeout(1), SourceBackGH.cmd()));
    SourceBackGH.done().onTrue((Commands.sequence(new Feed(), SourceToH.cmd())));
    SourceToH.done().onTrue(Commands.sequence(new Score().withTimeout(1), SourceBackGH.cmd()));
    SourceBackGH.done().onTrue((Commands.sequence(new Feed(), SourceToI.cmd())));
    SourceToI.done().onTrue(Commands.sequence(new Score().withTimeout(1), SourceBackIJ.cmd()));
    SourceBackIJ.done().onTrue((Commands.sequence(new Feed(), SourceToJ.cmd())));
    SourceToJ.done().onTrue(Commands.sequence(new Score().withTimeout(1), SourceBackIJ.cmd()));
    SourceBackIJ.done().onTrue((Commands.sequence(new Feed(), SourceToK.cmd())));
    SourceToK.done().onTrue(Commands.sequence(new Score().withTimeout(1), SourceBackKL.cmd()));
    SourceBackKL.done().onTrue((Commands.sequence(new Feed(), SourceToL.cmd())));
    SourceToL.done().onTrue(Commands.sequence(new Score().withTimeout(1), SourceBackKL.cmd()));

    return routine;
  }
}
