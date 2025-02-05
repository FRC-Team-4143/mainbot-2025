package frc.robot.autos;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.AutoManager;
import frc.robot.commands.Feed;
import frc.robot.commands.Score;
import frc.robot.commands.SetIdleMode;

public class ThreeCoralAuto {

  public static AutoRoutine getAutoRoutine() {

    AutoFactory autoFactory = AutoManager.getInstance().getAutoFactory();
    AutoRoutine routine = autoFactory.newRoutine("Three_Coral_Auto");
    AutoTrajectory MidcageToI = routine.trajectory("Midcage To I");
    AutoTrajectory SourceBackIJ = routine.trajectory("Source Back IJ");
    AutoTrajectory SourceToJ = routine.trajectory("Source To J");
    AutoTrajectory SourceToK = routine.trajectory("Source To K");
    AutoTrajectory SourceBackKL = routine.trajectory("Source Back KL");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                MidcageToI.cmd(),
                new SetIdleMode(),
                new Score().withTimeout(1),
                SourceBackIJ.cmd(),
                new SetIdleMode(),
                new Feed(),
                SourceToJ.cmd(),
                new SetIdleMode(),
                new Score().withTimeout(1),
                SourceBackIJ.cmd(),
                new SetIdleMode(),
                new Feed(),
                SourceToK.cmd(),
                new SetIdleMode(),
                new Score().withTimeout(1),
                SourceBackKL.cmd(),
                new SetIdleMode(),
                new Feed()));

    // routine.active().onTrue(Commands.sequence(SourceToA.cmd()));
    // SourceToA.done()
    // .onTrue(
    // Commands.sequence(new SetIdleMode(), new Score().withTimeout(1),
    // SourceBackAB.cmd()));
    // SourceBackAB.done().onTrue(Commands.sequence(new SetIdleMode(), new Feed(),
    // SourceToB.cmd()));
    // SourceToB.done()
    // .onTrue(
    // Commands.sequence(new SetIdleMode(), new Score().withTimeout(1),
    // SourceBackAB2.cmd()));
    // SourceBackAB2.done()
    // .onTrue((Commands.sequence(new SetIdleMode(), new Feed(), SourceToC.cmd())));
    // SourceToC.done()
    // .onTrue(
    // Commands.sequence(new SetIdleMode(), new Score().withTimeout(1),
    // SourceBackCD.cmd()));

    // SourceBackCD.done().onTrue((Commands.sequence(new SetIdleMode(), new Feed(),
    // SourceToD.cmd())));
    // SourceToD.done()
    // .onTrue(
    // Commands.sequence(new SetIdleMode(), new Score().withTimeout(1),
    // SourceBackCD2.cmd()));
    // SourceBackCD2.done()
    // .onTrue((Commands.sequence(new SetIdleMode(), new Feed(), SourceToE.cmd())));
    // SourceToE.done()
    // .onTrue(
    // Commands.sequence(new SetIdleMode(), new Score().withTimeout(1),
    // SourceBackEF.cmd()));
    // SourceBackEF.done().onTrue((Commands.sequence(new SetIdleMode(), new Feed(),
    // SourceToF.cmd())));
    // SourceToF.done()
    // .onTrue(
    // Commands.sequence(new SetIdleMode(), new Score().withTimeout(1),
    // SourceBackEF2.cmd()));
    // SourceBackEF2.done()
    // .onTrue((Commands.sequence(new SetIdleMode(), new Feed(), SourceToG.cmd())));
    // SourceToG.done()
    // .onTrue(
    // Commands.sequence(new SetIdleMode(), new Score().withTimeout(1),
    // SourceBackGH.cmd()));
    // SourceBackGH.done().onTrue((Commands.sequence(new SetIdleMode(), new Feed(),
    // SourceToH.cmd())));
    // SourceToH.done()
    // .onTrue(
    // Commands.sequence(new SetIdleMode(), new Score().withTimeout(1),
    // SourceBackGH2.cmd()));
    // SourceBackGH2.done()
    // .onTrue((Commands.sequence(new SetIdleMode(), new Feed(), SourceToI.cmd())));
    // SourceToI.done()
    // .onTrue(
    // Commands.sequence(new SetIdleMode(), new Score().withTimeout(1),
    // SourceBackIJ.cmd()));
    // SourceBackIJ.done().onTrue((Commands.sequence(new SetIdleMode(), new Feed(),
    // SourceToJ.cmd())));
    // SourceToJ.done()
    // .onTrue(
    // Commands.sequence(new SetIdleMode(), new Score().withTimeout(1),
    // SourceBackIJ2.cmd()));
    // SourceBackIJ2.done()
    // .onTrue((Commands.sequence(new SetIdleMode(), new Feed(), SourceToK.cmd())));
    // SourceToK.done()
    // .onTrue(
    // Commands.sequence(new SetIdleMode(), new Score().withTimeout(1),
    // SourceBackKL.cmd()));
    // SourceBackKL.done().onTrue((Commands.sequence(new SetIdleMode(), new Feed(),
    // SourceToL.cmd())));
    // SourceToL.done().onTrue(Commands.sequence(new SetIdleMode(), new
    // Score().withTimeout(1)));

    return routine;
  }
}
