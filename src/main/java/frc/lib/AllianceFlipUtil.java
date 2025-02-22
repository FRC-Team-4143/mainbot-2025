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
import frc.mw_lib.geometry.TightRope;

public class AllianceFlipUtil {

  public static Translation2d apply(Translation2d translation) {
    return translation.rotateAround(FieldConstants.FIELD_CENTER, Rotation2d.fromDegrees(180));
  }

  public static Pose2d apply(Pose2d pose) {
    return new Pose2d(
        apply(pose.getTranslation()), pose.getRotation().rotateBy(Rotation2d.fromDegrees(180)));
  }

  public static TightRope apply(TightRope tightRope) {
    tightRope.poseA = apply(tightRope.poseA);
    tightRope.poseB = apply(tightRope.poseB);
    Pose2d temp = tightRope.poseA;
    tightRope.poseA = tightRope.poseB;
    tightRope.poseB = temp;
    return tightRope;
  }
}
