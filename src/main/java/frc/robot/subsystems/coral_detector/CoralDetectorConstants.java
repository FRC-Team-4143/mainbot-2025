package frc.robot.subsystems.coral_detector;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class CoralDetectorConstants {
    public static final Transform3d BOT_TO_CAM_TRANSFORM =
        new Transform3d(
            Units.inchesToMeters(-8),
            Units.inchesToMeters(7),
            Units.inchesToMeters(28.25),
            new Rotation3d(
                Units.degreesToRadians(0),
                Units.degreesToRadians(18),
                Units.degreesToRadians(180 + 14)));
    public static final Translation2d BOT_TO_CAM_TRANSLATION =
        BOT_TO_CAM_TRANSFORM.getTranslation().toTranslation2d();
    public static final double CORAL_HEIGHT_METERS = Units.inchesToMeters(5);
    public static final double DETECTION_DISTANCE_LIMIT = 2.5;
    public static final double DETECT_DEBOUNCE_TIME_RISING = 0.1;
    public static final double DETECT_DEBOUNCE_TIME_FALLING = 0.5;

    public static final double DISPLAY_Z_OFFSET = Units.inchesToMeters(6);
    // public static final Rotation3d DISPLAY_ROTATION =
    // new Rotation3d(0, Units.degreesToRadians(90), 0);
    public static final Rotation3d DISPLAY_ROTATION =
        new Rotation3d(0, Units.degreesToRadians(0), 0);
  }
