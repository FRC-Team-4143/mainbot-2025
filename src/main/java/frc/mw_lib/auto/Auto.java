package frc.mw_lib.auto;

import choreo.auto.AutoRoutine;
import edu.wpi.first.math.trajectory.Trajectory;

public interface Auto {

  public AutoRoutine getAutoRoutine();

  public Trajectory[] getTrajectorys();
}
