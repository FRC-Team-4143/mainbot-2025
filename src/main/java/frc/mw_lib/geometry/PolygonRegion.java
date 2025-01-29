package frc.mw_lib.geometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import java.awt.geom.*;

/**
 * This class models a region of the field. It is defined by its vertices
 * Credit to frc-3061 for base code
 */
public class PolygonRegion implements Region {
  private Path2D shape;

  /**
   * Create a Region2d, a polygon, from an array of Translation2d specifying
   * vertices of a polygon.
   * The polygon is created using the even-odd winding rule.
   *
   * @param points     the array of Translation2d that define the vertices of the
   *                   region.
   * @param regionName the name of the region that is used for logging
   */
  public PolygonRegion(Translation2d[] points, String regionName) {
    this.shape = new Path2D.Double(Path2D.WIND_EVEN_ODD, points.length);
    this.shape.moveTo(points[0].getX(), points[0].getY());

    for (int i = 1; i < points.length; i++) {
      this.shape.lineTo(points[i].getX(), points[i].getY());
    }

    this.shape.closePath();
    logPoints(points, regionName);
  }

  /**
   * Log the bounding points of the region.
   * These can be visualized using AdvantageScope to confirm that the regions are
   * properly defined.
   */
  public void logPoints(Translation2d[] points, String regionName) {

    StructArrayPublisher<Translation2d> arrayPublisher = NetworkTableInstance.getDefault()
        .getStructArrayTopic(regionName, Translation2d.struct)
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

    return this.shape.contains(new Point2D.Double(other.getX(), other.getY()));
  }
}
