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
    double distanceFromFollowTarget =
        Math.abs(followDistance - current_translation.getDistance(path[0]));
    double LAST_distanceFromFollowTarget = distanceFromFollowTarget;
    int newTargetIndex = 0;

    // find nearest point on the path to the followDistance
    for (int i = 0; i < path.length; i++) {
      distanceFromFollowTarget =
          Math.abs(followDistance - current_translation.getDistance(path[0]));
      if (LAST_distanceFromFollowTarget < distanceFromFollowTarget) {
        newTargetIndex = i;
        break;
      }
      LAST_distanceFromFollowTarget = distanceFromFollowTarget;
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
