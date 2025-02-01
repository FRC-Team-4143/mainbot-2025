package frc.mw_lib.geometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

/** This class models a region of the field. Credit to frc-3061 for base code */
public interface Region {
  /**
   * Log the points of the region. These can be visualized using AdvantageScope to confirm that the
   * regions are properly defined.
   */
  public void logPoints(Translation2d[] points, String regionName);

  /** Returns true if the region contains a given Pose2d. */
  public boolean contains(Pose2d other);
}
