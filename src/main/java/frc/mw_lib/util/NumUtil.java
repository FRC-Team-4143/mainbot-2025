package frc.mw_lib.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public abstract class NumUtil {
  public static boolean epislonEquals(double x, double y, double epislon) {
    return Math.abs(x - y) < epislon;
  }

  public static boolean epislonEquals(double x, double y) {
    return epislonEquals(x, y, 1E-6);
  }

  public static boolean epislonEquals(Rotation2d x, Rotation2d y, double epislon) {
    return Math.abs(x.minus(y).getRadians()) < epislon;
  }

  public static boolean epislonEquals(Translation2d x, Translation2d y, double epislon) {
    return x.getDistance(y) < epislon;
  }

  public static boolean epislonEquals(
      Pose2d Pose2dA, Pose2d Pose2dB, double epislon_rotation, double epislon_translation) {
    return (epislonEquals(Pose2dA.getRotation(), Pose2dB.getRotation(), epislon_rotation)
        && epislonEquals(Pose2dA.getTranslation(), Pose2dB.getTranslation(), epislon_translation));
  }

  public static boolean epislonEquals(ChassisSpeeds ch1, ChassisSpeeds ch2, double d) {
    return epislonEquals(ch1.vxMetersPerSecond, ch2.vxMetersPerSecond, d)
        && epislonEquals(ch1.vyMetersPerSecond, ch2.vyMetersPerSecond, d);
  }

  public static double clamp(double x, double range) {
    if (x > range) x = range;
    if (x < -range) x = -range;
    return x;
  }

  public static double clamp(double x, double min, double max) {
    if (x > max) return max;
    if (x < min) return min;
    return x;
  }

  public static Transform2d flatten(Transform3d tf) {
    return new Transform2d(tf.getX(), tf.getY(), tf.getRotation().toRotation2d());
  }
}
