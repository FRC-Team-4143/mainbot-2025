package frc.mw_lib.geometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import java.awt.geom.*;

/**
 * This class models a region of the field. It is defined by its vertices Credit to frc-3061 for
 * base code
 */
public class PolygonRegion implements Region {
  private Path2D shape_;
  private String name_;
  private Translation2d[] points_;
  private StructArrayPublisher<Translation2d> array_publisher_;

  /**
   * Create a Region2d, a polygon, from an array of Translation2d specifying vertices of a polygon.
   * The polygon is created using the even-odd winding rule.
   *
   * @param points the array of Translation2d that define the vertices of the region.
   * @param regionName the name of the region that is used for logging
   */
  public PolygonRegion(Translation2d[] points, String regionName) {
    name_ = regionName;
    points_ = points;
    array_publisher_ =
        NetworkTableInstance.getDefault()
            .getStructArrayTopic("Regions/" + name_, Translation2d.struct)
            .publish();
    constructRegion();
  }

  public void constructRegion() {
    shape_ = new Path2D.Double(Path2D.WIND_EVEN_ODD, points_.length);
    shape_.moveTo(points_[0].getX(), points_[0].getY());

    for (int i = 1; i < points_.length; i++) {
      shape_.lineTo(points_[i].getX(), points_[i].getY());
    }
    shape_.closePath();
    logPoints();
  }

  /**
   * Log the bounding points of the region. These can be visualized using AdvantageScope to confirm
   * that the regions are properly defined.
   */
  public void logPoints() {
    array_publisher_.set(points_);
  }

  /**
   * Returns true if the region contains a given Pose2d.
   *
   * @param other the given pose2d
   * @return if the pose is inside the region
   */
  public boolean contains(Pose2d other) {
    return shape_.contains(new Point2D.Double(other.getX(), other.getY()));
  }

  public String getName() {
    return name_;
  }

  public Translation2d[] getPoints() {
    Translation2d[] saftey = points_;
    return saftey;
  }
}
