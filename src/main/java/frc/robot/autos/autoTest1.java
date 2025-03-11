package frc.robot.autos;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.mw_lib.auto.Auto;
import frc.robot.AutoManager;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.SwerveDrivetrain.DriveMode;
import java.util.ArrayList;

public class autoTest1 implements Auto {

  private static ArrayList<AutoTrajectory> autoTrajectorys = new ArrayList<>();

  public AutoRoutine getAutoRoutine() {

    AutoFactory autoFactory = AutoManager.getInstance().getAutoFactory();
    AutoRoutine routine = autoFactory.newRoutine("autoTest1");
    AutoTrajectory driveforward = routine.trajectory("Move Forward");
    autoTrajectorys.add(driveforward);

    routine.active().onTrue(Commands.sequence(driveforward.resetOdometry(), driveforward.cmd()));
    driveforward
        .done()
        .onTrue(
            Commands.runOnce(() -> SwerveDrivetrain.getInstance().setDriveMode(DriveMode.IDLE)));

    return routine;
  }

  public Trajectory[] getTrajectorys() {
    return new Trajectory[1];
  }
}
