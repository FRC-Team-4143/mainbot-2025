package frc.lib;

import edu.wpi.first.math.geometry.Translation2d;
import frc.lib.ElevatorKinematics.JointSpaceTarget;
import frc.mw_lib.geometry.spline.SplineUtil;

public class ElevatorPlanner {
  private ElevatorKinematics kinematics;
  private Translation2d[] path;
  private double subdivisionsPerUnit;
  private double followDistance;

  public ElevatorPlanner(ElevatorKinematics k, double subsPerUnit, double followDist) {
    this.kinematics = k;
    this.subdivisionsPerUnit = subsPerUnit;
    this.followDistance = followDist;
  }

  public void plan(Translation2d[] wayPoints) {
    path = SplineUtil.subdividePoints(wayPoints, subdivisionsPerUnit);
  }

  public JointSpaceTarget nextTarget(Translation2d current_translation) {
    double distanceFromFolowTarget =
        Math.abs(followDistance - current_translation.getDistance(path[0]));
    double LAST_distanceFromFolowTarget = distanceFromFolowTarget;
    int newTargetIndex = -1;

    // find nearest point on the path to the followDistance
    for (int i = 0; i < path.length; i++) {
      distanceFromFolowTarget = Math.abs(followDistance - current_translation.getDistance(path[0]));
      if (LAST_distanceFromFolowTarget < distanceFromFolowTarget) {
        newTargetIndex = i;
        break;
      }
      LAST_distanceFromFolowTarget = distanceFromFolowTarget;
    }

    // remove any points that are behind the target point
    Translation2d[] newPath = new Translation2d[path.length - newTargetIndex];
    for (int i = newTargetIndex; i < path.length; i++) {
      newPath[i - newTargetIndex] = path[i];
    }
    path = newPath;

    return kinematics.translationToJointSpace(path[0]);
  }
}
