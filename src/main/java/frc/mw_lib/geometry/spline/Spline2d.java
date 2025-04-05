package frc.mw_lib.geometry.spline;

import edu.wpi.first.math.geometry.Translation2d;

public class Spline2d {
  private Spline splineXVals, splineYVals;

  public Spline2d(Translation2d p0, Translation2d p1, Translation2d p2, Translation2d p3) {
    splineXVals = new Spline(p0.getX(), p1.getX(), p2.getX(), p3.getX());
    splineYVals = new Spline(p0.getY(), p1.getY(), p2.getY(), p3.getY());
  }

  public Translation2d q(float t) {
    return new Translation2d(splineXVals.q(t), splineYVals.q(t));
  }
}
