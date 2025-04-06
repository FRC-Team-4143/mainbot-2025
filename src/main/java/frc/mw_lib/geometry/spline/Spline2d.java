package frc.mw_lib.geometry.spline;

import edu.wpi.first.math.geometry.Translation3d;

public class Spline2d {
  private Spline splineXVals, splineYVals;

  public Spline2d(Translation3d p0, Translation3d p1, Translation3d p2, Translation3d p3) {
    splineXVals = new Spline(p0.getX(), p1.getX(), p2.getX(), p3.getX());
    splineYVals = new Spline(p0.getZ(), p1.getZ(), p2.getZ(), p3.getZ());
  }

  public Translation3d q(float t) {
    return new Translation3d(splineXVals.q(t), 0, splineYVals.q(t));
  }
}
