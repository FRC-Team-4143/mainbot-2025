package frc.robot.autos;

import frc.lib.Util;
import frc.robot.AutoManager;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.Auto;
import choreo.auto.AutoFactory;

public class autoTest1 {

    public static AutoRoutine getAutoRoutine() {

        AutoFactory autoFactory = AutoManager.getInstance().getAutoFactory();
        AutoRoutine routine = autoFactory.newRoutine("autoTest1");
        AutoTrajectory driveforward = routine.trajectory("Move Forward");

        routine.active().onTrue(
                Commands.sequence(
                        driveforward.resetOdometry(),
                        driveforward.cmd()));

        return routine;
    }

}
