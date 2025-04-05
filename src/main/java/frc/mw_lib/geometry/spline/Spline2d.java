package frc.mw_lib.geometry.spline;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.spline.Spline.ControlVector;

public class Spline2d {
  private Spline splineXVals, splineYVals;

  public Spline2d(ControlVector cv) {
    splineXVals = new Spline(cv.x[0], cv.x[1], cv.x[2], cv.x[3]);
    splineYVals = new Spline(cv.y[0], cv.y[1], cv.y[2], cv.y[3]);
  }

  public Translation2d q(float t) {
    return new Translation2d(splineXVals.q(t), splineYVals.q(t));
  }
}
