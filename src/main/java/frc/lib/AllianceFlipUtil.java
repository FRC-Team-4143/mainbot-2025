// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.lib;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.mw_lib.geometry.CircularRegion;
import frc.mw_lib.geometry.PolygonRegion;
import frc.mw_lib.geometry.Region;
import frc.mw_lib.geometry.TightRope;

public class AllianceFlipUtil {

  public enum SymmetryType {
    DIAGONAL,
    DIRECT
  }

  public static Translation2d apply(Translation2d translation, SymmetryType symmetry) {
    if (symmetry == SymmetryType.DIAGONAL) {
      return translation.rotateAround(FieldConstants.FIELD_CENTER, Rotation2d.fromDegrees(180));
    } else {
      return translation.plus(
          new Translation2d(
              translation.getDistance(
                      new Translation2d(FieldConstants.FIELD_CENTER.getX(), translation.getY()))
                  * 2,
              translation.getY()));
    }
  }

  public static Pose2d apply(Pose2d pose, SymmetryType symmetry) {
    if (symmetry == SymmetryType.DIAGONAL) {
      return new Pose2d(
          apply(pose.getTranslation(), symmetry),
          pose.getRotation().rotateBy(Rotation2d.fromDegrees(180)));
    } else {
      return new Pose2d(apply(pose.getTranslation(), symmetry), pose.getRotation());
    }
  }

  public static Region apply(Region region, SymmetryType symmetry) {
    if (region instanceof PolygonRegion) {
      return apply((PolygonRegion) region, symmetry);
    } else {
      return apply((CircularRegion) region, symmetry);
    }
  }

  public static PolygonRegion apply(PolygonRegion region, SymmetryType symmetry) {
    Translation2d[] points = region.getPoints();
    for (int i = 0; i < points.length; i++) {
      points[i] = AllianceFlipUtil.apply(points[i], symmetry);
    }
    return new PolygonRegion(points, region.getName());
  }

  public static CircularRegion apply(CircularRegion region, SymmetryType symmetry) {
    Translation2d newCenter;
    if (symmetry == SymmetryType.DIAGONAL) {
      newCenter = apply(region.getCenter(), SymmetryType.DIAGONAL);
    } else {
      newCenter = apply(region.getCenter(), SymmetryType.DIRECT);
    }
    return new CircularRegion(newCenter, region.getRadius(), region.getName());
  }

  public static TightRope apply(TightRope tightRope, SymmetryType symmetry) {
    if (symmetry == SymmetryType.DIAGONAL) {
      return new TightRope(
          AllianceFlipUtil.apply(tightRope.poseB, symmetry),
          AllianceFlipUtil.apply(tightRope.poseA, symmetry),
          tightRope.getName());
    } else {
      return new TightRope(
          AllianceFlipUtil.apply(tightRope.poseB, symmetry),
          AllianceFlipUtil.apply(tightRope.poseA, symmetry),
          tightRope.getName());
    }
  }
}
