package frc.mw_lib.geometry;

import edu.wpi.first.math.geometry.Pose2d;

public class TightRope {
  public Pose2d poseA;
  public Pose2d poseB;

  public TightRope(Pose2d a, Pose2d b) {
    poseA = a;
    poseB = b;
  }
}
