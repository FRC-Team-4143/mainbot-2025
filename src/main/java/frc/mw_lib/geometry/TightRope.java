package frc.mw_lib.geometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;

public class TightRope {
  public Pose2d poseA;
  public Pose2d poseB;
  private StructArrayPublisher<Translation2d> array_publisher_;
  private String name;

  public TightRope(Pose2d a, Pose2d b, String n) {
    poseA = a;
    poseB = b;
    name = n;

    array_publisher_ =
        NetworkTableInstance.getDefault()
            .getStructArrayTopic("TightRopes/" + name, Translation2d.struct)
            .publish();

    logPoints();
  }

  public void logPoints() {
    Translation2d[] points_ = new Translation2d[2];
    points_[0] = poseA.getTranslation();
    points_[1] = poseB.getTranslation();
    array_publisher_.set(points_);
  }

  public String getName() {
    return name;
  }
}
