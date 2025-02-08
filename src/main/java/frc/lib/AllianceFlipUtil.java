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

public class AllianceFlipUtil {

  public static Translation2d apply(Translation2d translation, boolean flip) {
    if (flip) {
      return translation.rotateAround(FieldConstants.FIELD_CENTER, Rotation2d.fromDegrees(180));
    }
    return translation;
  }

  public static Pose2d apply(Pose2d pose, boolean flip) {
    if (flip) {
      return new Pose2d(
          apply(pose.getTranslation(), flip),
          pose.getRotation().rotateBy(Rotation2d.fromDegrees(180)));
    }
    return pose;
  }
}
