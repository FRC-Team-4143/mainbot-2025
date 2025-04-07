package frc.mw_lib.geometry.spline;

import edu.wpi.first.math.geometry.Translation3d;
import java.util.ArrayList;

public class SplineUtil {
  /**
   * Creates catmull spline curves between the points array.
   *
   * @param points The current 2D points array
   * @param subdivisions The number of subdivisions to add between each of the points.
   * @return A larger array with the points subdivided.
   */
  public static ArrayList<Translation3d> subdividePoints(
      ArrayList<Translation3d> points, ArrayList<Integer> subdivisions) {
    ArrayList<Translation3d> subdividedPoints = new ArrayList<>();
    for (int i = 0; i < points.size() - 1; i++) {
      float increments = 1f / (float) subdivisions.get(i);

      Translation3d p0 = i == 0 ? points.get(i) : points.get(i - 1);
      Translation3d p1 = points.get(i);
      Translation3d p2 = points.get(i + 1);
      Translation3d p3 = (i + 2 == points.size()) ? points.get(i + 1) : points.get(i + 2);

      Spline2d crs = new Spline2d(p0, p1, p2, p3);

      for (int j = 0; j < subdivisions.get(i); j++) {
        subdividedPoints.add(crs.q(j * increments));
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
  public static ArrayList<Translation3d> subdividePoints(
      ArrayList<Translation3d> points, double subdivisionsPerUnit) {

    ArrayList<Double> distance_between_points = new ArrayList<>();
    for (int i = 0; i < points.size() - 1; i++) {
      distance_between_points.add(points.get(i).getDistance(points.get(i + 1)));
    }

    ArrayList<Integer> subdivisions = new ArrayList<>();
    for (int i = 0; i < points.size() - 1; i++) {
      subdivisions.add((int) Math.round(distance_between_points.get(i) * subdivisionsPerUnit));
    }

    return subdividePoints(points, subdivisions);
  }
}
