package frc.mw_lib.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.AllianceFlipUtil;
import java.util.ArrayList;
import java.util.List;

public class Auto extends SequentialCommandGroup {

  protected ArrayList<Pose2d[]> trajectory_list_ = new ArrayList<>();

  public List<Pose2d> getPath() {
    ArrayList<Pose2d> path_poses = new ArrayList<>();
    for (Pose2d[] trajectory : trajectory_list_) {
      for (Pose2d pose : trajectory) {
        DriverStation.getAlliance()
            .ifPresentOrElse(
                (alliance) -> {
                  if (alliance == Alliance.Red) {
                    path_poses.add(AllianceFlipUtil.apply(pose));
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
