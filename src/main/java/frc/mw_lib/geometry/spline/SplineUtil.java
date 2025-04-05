package frc.mw_lib.geometry.spline;

import edu.wpi.first.math.geometry.Translation2d;

public class SplineUtil {
  /**
   * Creates catmull spline curves between the points array.
   *
   * @param points The current 2D points array
   * @param subdivisions The number of subdivisions to add between each of the points.
   * @return A larger array with the points subdivided.
   */
  public static Translation2d[] subdividePoints(Translation2d[] points, int[] subdivisions) {
    int totalSubdivisions = sumArray(subdivisions);
    Translation2d[] subdividedPoints =
        new Translation2d[((points.length - 1) * totalSubdivisions) + 1];

    for (int i = 0; i < points.length - 1; i++) {
      float increments = 1f / (float) subdivisions[i];

      Translation2d p0 = i == 0 ? points[i] : points[i - 1];
      Translation2d p1 = points[i];
      Translation2d p2 = points[i + 1];
      Translation2d p3 = (i + 2 == points.length) ? points[i + 1] : points[i + 2];

      Spline2d crs = new Spline2d(p0, p1, p2, p3);

      for (int j = 0; j <= subdivisions[i]; j++) {
        subdividedPoints[(i * subdivisions[i]) + j] = crs.q(j * increments);
      }
    }

    return subdividedPoints;
  }

  /**
   * Creates catmull spline curves between the points array with a given number for subdivions per
   * unit.
   *
   * @param points The current 2D points array
   * @param subdivisionsPerUnit The number of subdivisions to add per unit of liner distatnce
   * @return A larger array with the points subdivided.
   */
  public static Translation2d[] subdividePoints(
      Translation2d[] points, double subdivisionsPerUnit) {
    double[] distOfPoints = new double[points.length - 1];
    for (int i = 0; i < distOfPoints.length - 1; i++) {
      distOfPoints[i] = points[i].getDistance(points[i + 1]);
    }

    int[] subdivisions = new int[points.length - 1];
    for (int i = 0; i < subdivisions.length; i++) {
      subdivisions[i] = (int) Math.round(distOfPoints[i] * subdivisionsPerUnit);
    }

    int totalSubdivisions = sumArray(subdivisions);
    Translation2d[] subdividedPoints = new Translation2d[totalSubdivisions + 1];
    subdividedPoints = subdividePoints(points, subdivisions);

    return subdividedPoints;
  }

  private static int sumArray(int[] a) {
    int count = 0;
    for (int i = 0; i < a.length; i++) {
      count += a[i];
    }
    return count;
  }
}
