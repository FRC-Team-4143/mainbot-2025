package frc.lib;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import frc.lib.ElevatorKinematics.JointSpaceSolution;
import frc.lib.ElevatorKinematics.SolutionType;
import frc.mw_lib.geometry.spline.SplineUtil;
import frc.mw_lib.geometry.spline.Waypoint;
import java.util.ArrayList;
import java.util.Iterator;

public class ElevatorPlanner {

  private ElevatorKinematics kinematics_;
  private ArrayList<Waypoint> path_ = new ArrayList<>();
  private double subdivisions_per_unit_ = 0;
  private double follow_distance_ = 0;
  private static final double REQUIRED_WAYPOIT_TOLERENCE = Units.inchesToMeters(1);
  private static double MAX_ITERATIONS;

  private StructArrayPublisher<Translation3d> path_publisher_;
  private StructArrayPublisher<Translation3d> point_publisher_;
  private StructPublisher<Pose2d> base_publisher_;
  private StructPublisher<Translation3d> target_publisher_;
  private BooleanPublisher required_target_pub_;

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
    required_target_pub_ =
        NetworkTableInstance.getDefault()
            .getBooleanTopic("ElevatorPlanner/required_target")
            .publish();
    base_publisher_ =
        NetworkTableInstance.getDefault()
            .getStructTopic("Components/Static Base", Pose2d.struct)
            .publish();
    base_publisher_.set(new Pose2d());
  }

  public void plan(ArrayList<Waypoint> waypoints) {
    System.out.println("Started Plan build waypoints size:" + waypoints.size());
    Translation3d[] waypoint_translation_array = new Translation3d[waypoints.size()];
    for (int i = 0; i < waypoint_translation_array.length; i++) {
      waypoint_translation_array[i] = waypoints.get(i).translation;
    }
    point_publisher_.set(waypoint_translation_array);

    path_ = SplineUtil.subdividePoints(waypoints, subdivisions_per_unit_);

    Translation3d[] path_array = new Translation3d[path_.size()];
    for (int j = 0; j < path_array.length - 1; j++) {
      Waypoint tmp = path_.get(j);
      tmp.translation = kinematics_.constrainReachableTranslation(tmp.translation);
      path_array[j] = tmp.translation;
      path_.set(j, tmp);
    }
    path_publisher_.set(path_array);
    System.out.println("path_array size:" + path_array.length);
    System.out.println("waypoint_translation_array size:" + waypoint_translation_array.length);
  }

  public boolean hasPath() {
    return path_.size() > 1;
  }

  public synchronized JointSpaceSolution nextTarget(
      Translation3d current_translation, SolutionType st) {
    Iterator<Waypoint> iterator = path_.iterator();
    int iterations = 0;
    while (iterator.hasNext()) {
      iterations++;
      Waypoint waypoint = iterator.next();
      double dist_to_way = current_translation.getDistance(waypoint.translation);

      if (waypoint.required && dist_to_way > REQUIRED_WAYPOIT_TOLERENCE) {
        break;
      }

      if (dist_to_way >= follow_distance_ || path_.size() == 1) {
        break;
      }

      iterator.remove();

      if (iterations > MAX_ITERATIONS) {
        System.out.println("Next Target Overrun!");
        break;
      }
    }

    target_publisher_.set(path_.get(0).translation);
    required_target_pub_.set(path_.get(0).required);
    return kinematics_.translationToJointSpace(path_.get(0).translation, st);
  }
}
