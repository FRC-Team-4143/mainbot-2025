package frc.lib;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import frc.lib.ElevatorKinematics.JointSpaceTarget;
import frc.mw_lib.geometry.spline.SplineUtil;
import java.util.ArrayList;
import java.util.Arrays;

public class ElevatorPlanner {
  private ElevatorKinematics kinematics_;
  private ArrayList<Translation3d> path_ = new ArrayList<>();
  private double subdivisions_per_unit_ = 0;
  private double follow_distance_ = 0;

  private StructArrayPublisher<Translation3d> path_publisher_;
  private StructArrayPublisher<Translation3d> point_publisher_;
  private StructPublisher<Pose2d> base_publisher_;

  public ElevatorPlanner(ElevatorKinematics k, double subsPerUnit, double followDist) {
    this.kinematics_ = k;
    this.subdivisions_per_unit_ = subsPerUnit;
    this.follow_distance_ = followDist;
    path_publisher_ =
        NetworkTableInstance.getDefault()
            .getStructArrayTopic("ElevatorPlanner/Path", Translation3d.struct)
            .publish();
    point_publisher_ =
        NetworkTableInstance.getDefault()
            .getStructArrayTopic("ElevatorPlanner/Points", Translation3d.struct)
            .publish();
    base_publisher_ =
        NetworkTableInstance.getDefault()
            .getStructTopic("Components/Static Base", Pose2d.struct)
            .publish();
    base_publisher_.set(new Pose2d());
  }

  public void plan(Translation3d[] wayPoints) {
    point_publisher_.set(wayPoints);
    path_ =
        new ArrayList<>(
            Arrays.asList(SplineUtil.subdividePoints(wayPoints, subdivisions_per_unit_)));
    Translation3d[] path_array = new Translation3d[path_.size()];
    path_publisher_.set(path_.toArray(path_array));
  }

  public boolean hasPath() {
    return path_.size() > 0;
  }

  public JointSpaceTarget nextTarget(Translation3d current_translation) {
    double distanceFromFollowTarget = 0;
    double LAST_distanceFromFollowTarget = 10;
    int newTargetIndex = 0;

    // find nearest point on the path to the followDistance
    for (int i = 0; i < path_.size(); i++) {
      distanceFromFollowTarget =
          Math.abs(follow_distance_ - current_translation.getDistance(path_.get(i)));
      if (LAST_distanceFromFollowTarget < distanceFromFollowTarget) {
        newTargetIndex = i;
        break;
      }
      LAST_distanceFromFollowTarget = distanceFromFollowTarget;
    }

    // remove any points that are behind the target point
    path_ = new ArrayList<>(path_.subList(newTargetIndex, path_.size() - 1));
    System.out.println("path_ Size:" + path_.size());

    return kinematics_.translationToJointSpace(path_.get(0));
  }
}
