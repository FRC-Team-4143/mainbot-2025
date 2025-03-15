package frc.mw_lib.util;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.IOException;

public abstract class TagLayouts {
  public static AprilTagFieldLayout getTagLayoutFromPath(String path) {
    AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    try {
      layout = new AprilTagFieldLayout(Filesystem.getDeployDirectory().toPath().resolve(path));
    } catch (IOException E) {
      DriverStation.reportWarning(
          "Unable to find path to aprilTagFeild, k2025ReefscapeWelded used", false);
    }
    return layout;
  }
}
