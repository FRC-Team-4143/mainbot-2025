package frc.mw_lib.geometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;

/**
 * This class models a region of the field. It is defined by its center and radius Credit to
 * frc-3061 for base code
 */
public class CircularRegion implements Region {
  private Translation2d center_;
  private String name_;
  private double radius_;
  private StructArrayPublisher<Translation2d> array_publisher_;

  /**
   * Create a Region2d, a polygon, from an array of Translation2d specifying vertices of a polygon.
   * The polygon is created using the even-odd winding rule.
   *
   * @param points the array of Translation2d that define the vertices of the region.
   * @param regionName the name of the region that is used for logging
   */
  public CircularRegion(Translation2d center, double radius, String region_name) {
    radius_ = radius;
    name_ = region_name;
    center_ = center;
    array_publisher_ =
        NetworkTableInstance.getDefault()
            .getStructArrayTopic("Regions/" + name_, Translation2d.struct)
            .publish();
    constructRegion();
  }

  public void constructRegion() {
    logPoints();
  }

  /**
   * Log the circumference of the circle in 360 points. These can be visualized using AdvantageScope
   * to confirm that the regions are properly defined.
   */
  public void logPoints() {
    Translation2d[] points = new Translation2d[360];
    for (int i = 0; i < 360; i++) {
      points[i] =
          center_
              .plus(new Translation2d(radius_, 0))
              .rotateAround(center_, Rotation2d.fromDegrees(i));
    }
    array_publisher_.set(points);
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

  public String getName() {
    return name_;
  }

  public Translation2d getCenter() {
    return center_;
  }

  public double getRadius() {
    return radius_;
  }
}
