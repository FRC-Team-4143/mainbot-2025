package frc.lib;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import frc.lib.ElevatorKinematics.JointSpaceTarget;
import frc.mw_lib.geometry.spline.SplineUtil;
import java.util.ArrayList;
import java.util.Iterator;

public class ElevatorPlanner {

  private ElevatorKinematics kinematics_;
  private ArrayList<Translation3d> path_ = new ArrayList<>();
  private double subdivisions_per_unit_ = 0;
  private double follow_distance_ = 0;
  private static double MAX_ITERATIONS;

  private StructArrayPublisher<Translation3d> path_publisher_;
  private StructArrayPublisher<Translation3d> point_publisher_;
  private StructPublisher<Pose2d> base_publisher_;
  private StructPublisher<Translation3d> target_publisher_;

  public ElevatorPlanner(
      ElevatorKinematics kinematics, double subdivisions_per_unit, double follow_distance) {
    kinematics_ = kinematics;
    subdivisions_per_unit_ = subdivisions_per_unit;
    MAX_ITERATIONS = subdivisions_per_unit * 5;
    follow_distance_ = follow_distance;
    path_publisher_ =
        NetworkTableInstance.getDefault()
            .getStructArrayTopic("ElevatorPlanner/Path", Translation3d.struct)
            .publish();
    point_publisher_ =
        NetworkTableInstance.getDefault()
            .getStructArrayTopic("ElevatorPlanner/Points", Translation3d.struct)
            .publish();
    target_publisher_ =
        NetworkTableInstance.getDefault()
            .getStructTopic("ElevatorPlanner/Target", Translation3d.struct)
            .publish();
    base_publisher_ =
        NetworkTableInstance.getDefault()
            .getStructTopic("Components/Static Base", Pose2d.struct)
            .publish();
    base_publisher_.set(new Pose2d());
  }

  public void plan(ArrayList<Translation3d> waypoints) {
    Translation3d[] waypoints_array = new Translation3d[waypoints.size()];
    point_publisher_.set(waypoints.toArray(waypoints_array));
    path_ = SplineUtil.subdividePoints(waypoints, subdivisions_per_unit_);
    Translation3d[] path_array = new Translation3d[path_.size()];
    path_publisher_.set(path_.toArray(path_array));
  }

  public boolean hasPath() {
    return !path_.isEmpty();
  }

  public JointSpaceTarget nextTarget(Translation3d current_translation) {
    Iterator<Translation3d> iterator = path_.iterator();
    int iterations = 0;
    while (iterator.hasNext()) {
      iterations++;
      Translation3d translation = iterator.next();
      if (current_translation.getDistance(translation) < follow_distance_ && path_.size() > 1) {
        iterator.remove();
      } else {
        break;
      }
      if (iterations > MAX_ITERATIONS) {
        System.out.println("Next Target Overrun!");
        break;
      }
    }
    target_publisher_.set(path_.get(0));
    return kinematics_.translationToJointSpace(path_.get(0));
  }
}
