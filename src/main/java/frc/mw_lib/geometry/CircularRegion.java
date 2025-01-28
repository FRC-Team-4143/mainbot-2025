package frc.mw_lib.geometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import java.awt.geom.*;

/**
 * This class models a region of the field. It is defined by its vertices and the transition points
 * to neighboring regions.
 */
public class CircularRegion {
  private Path2D shape_;
  private Translation2d center_;
  private double radius_;
  private String region_name_;

  /**
   * Create a Region2d, a polygon, from an array of Translation2d specifying vertices of a polygon.
   * The polygon is created using the even-odd winding rule.
   *
   * @param points the array of Translation2d that define the vertices of the region.
   * @param regionName the name of the region that is used for logging
   */
  public CircularRegion(Translation2d center_, double radius_, String region_name_) {
    this.center_ = center_;
    this.radius_ = radius_;
    this.region_name_ = region_name_;

    logPoints();
  }

  /**
   * Log the bounding rectangle of the region and the transition points to neighboring regions.
   * These can be visualized using AdvantageScope to confirm that the regions are properly defined.
   */
  public void logPoints() {
    Translation2d[] points = new Translation2d[360];
    for (int i = 0; i < 360; i++) {
      points[i] =
          center_
              .plus(new Translation2d(radius_, 0))
              .rotateAround(center_, Rotation2d.fromDegrees(i));
    }
    StructArrayPublisher<Translation2d> arrayPublisher =
        NetworkTableInstance.getDefault()
            .getStructArrayTopic(region_name_, Translation2d.struct)
            .publish();
    arrayPublisher.set(points);
  }

  /**
   * Returns true if the region contains a given Pose2d.
   *
   * @param other the given pose2d
   * @return if the pose is inside the region
   */
  public boolean contains(Pose2d other) {
    return center_.getDistance(other.getTranslation()) < radius_;
  }
}
