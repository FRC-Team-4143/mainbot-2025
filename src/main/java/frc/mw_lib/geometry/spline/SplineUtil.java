package frc.mw_lib.geometry.spline;

import edu.wpi.first.math.geometry.Translation3d;

public class SplineUtil {
  /**
   * Creates catmull spline curves between the points array.
   *
   * @param points The current 2D points array
   * @param subdivisions The number of subdivisions to add between each of the points.
   * @return A larger array with the points subdivided.
   */
  public static Translation3d[] subdividePoints(Translation3d[] points, int[] subdivisions) {
    int totalSubdivisions = sumArray(subdivisions);
    Translation3d[] subdividedPoints = new Translation3d[totalSubdivisions];
    int index = 0;
    for (int i = 0; i < points.length - 1; i++) {
      float increments = 1f / (float) subdivisions[i];

      Translation3d p0 = i == 0 ? points[i] : points[i - 1];
      Translation3d p1 = points[i];
      Translation3d p2 = points[i + 1];
      Translation3d p3 = (i + 2 == points.length) ? points[i + 1] : points[i + 2];

      Spline2d crs = new Spline2d(p0, p1, p2, p3);

      for (int j = 0; j < subdivisions[i]; j++) {
        subdividedPoints[index + j] = crs.q(j * increments);
      }
      index += subdivisions[i];
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
  public static Translation3d[] subdividePoints(
      Translation3d[] points, double subdivisionsPerUnit) {

    double[] distOfPoints = new double[points.length - 1];
    for (int i = 0; i < distOfPoints.length; i++) {
      distOfPoints[i] = points[i].getDistance(points[i + 1]);
    }

    int[] subdivisions = new int[points.length - 1];
    for (int i = 0; i < subdivisions.length; i++) {
      subdivisions[i] = (int) Math.round(distOfPoints[i] * subdivisionsPerUnit);
    }

    return subdividePoints(points, subdivisions);
  }

  private static int sumArray(int[] a) {
    int count = 0;
    for (int i = 0; i < a.length; i++) {
      count += a[i];
    }
    return count;
  }
}
