package frc.mw_lib.auto;

import choreo.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.AllianceFlipUtil;
import frc.lib.FieldConstants;
import frc.mw_lib.command.NoReqSequentialCommandGroup;
import frc.mw_lib.subsystem.Subsystem;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;

public class Auto extends NoReqSequentialCommandGroup {

  private ArrayList<Pose2d[]> trajectory_list_ = new ArrayList<>();
  protected LinkedHashMap<String, Trajectory<?>> trajectories = new LinkedHashMap<>();

  public Auto() {
    this.setName(getClass().getSimpleName());
  }

  protected void loadTrajectory(String name) {
    Trajectory<?> traj = choreo.Choreo.loadTrajectory(name).get();

    trajectory_list_.add(traj.getPoses());
    if (!trajectories.containsKey(name)) trajectories.put(name, traj);
  }

  protected Command getTrajectoryCmd(String Name) {
    Trajectory<?> traj = trajectories.get(Name);
    return AutoManager.getInstance().getAutoFactory().trajectoryCmd(traj);
  }

  protected Command getTrajectoryCmd(String Name, Subsystem... requirements) {
    Trajectory<?> traj = trajectories.get(Name);
    Command cmd = AutoManager.getInstance().getAutoFactory().trajectoryCmd(traj);
    cmd.addRequirements(requirements);
    return cmd;
  }

  public List<Pose2d> getPath() {
    ArrayList<Pose2d> path_poses = new ArrayList<>();
    for (Pose2d[] trajectory : trajectory_list_) {
      for (Pose2d pose : trajectory) {
        DriverStation.getAlliance()
            .ifPresentOrElse(
                (alliance) -> {
                  if (alliance == Alliance.Red) {
                    path_poses.add(AllianceFlipUtil.apply(pose, FieldConstants.SYMMETRY_TYPE));
                  } else {
                    path_poses.add(pose);
                  }
                  // Default to Blue when no alliance present
                },
                () -> path_poses.add(pose));
      }
    }
    return path_poses;
  }
}
