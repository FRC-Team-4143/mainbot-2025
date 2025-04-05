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
  public static Translation2d[] subdividePoints(Translation2d[] points, int subdivisions) {
    Translation2d[] subdividedPoints = new Translation2d[((points.length - 1) * subdivisions) + 1];

    float increments = 1f / (float) subdivisions;

    for (int i = 0; i < points.length - 1; i++) {
      Translation2d p0 = i == 0 ? points[i] : points[i - 1];
      Translation2d p1 = points[i];
      Translation2d p2 = points[i + 1];
      Translation2d p3 = (i + 2 == points.length) ? points[i + 1] : points[i + 2];

      Spline2d crs = new Spline2d(p0, p1, p2, p3);

      for (int j = 0; j <= subdivisions; j++) {
        subdividedPoints[(i * subdivisions) + j] = crs.q(j * increments);
      }
    }

    return subdividedPoints;
  }

//   public static void main(String[] args) {
//     Translation2d[] pointArray = new Translation2d[4];

//     pointArray[0] = new Translation2d(1f, 1f);
//     pointArray[1] = new Translation2d(2f, 2f);
//     pointArray[2] = new Translation2d(3f, 2f);
//     pointArray[3] = new Translation2d(4f, 1f);

//     Translation2d[] subdividedPoints = SplineUtil.subdividePoints(pointArray, 4);

//     for (Translation2d point : subdividedPoints) {
//       System.out.println("" + point);
//     }
//   }
}
