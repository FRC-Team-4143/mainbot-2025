package frc.lib;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.lib.FieldConstants.Barge;
import frc.mw_lib.geometry.CircularRegion;
import frc.mw_lib.geometry.PolygonRegion;
import frc.mw_lib.geometry.Region;
import frc.robot.Constants;
import java.util.Hashtable;

public class FieldRegions {

  public static PolygonRegion PROCESSOR_REGION =
      new PolygonRegion(
          new Translation2d[] {
            new Translation2d(4, 0),
            new Translation2d(4, 3),
            new Translation2d(8, 3),
            new Translation2d(8, 0),
            new Translation2d(4, 0),
          },
          "Processor");
  public static PolygonRegion PROCESSOR_DEAD_REGION =
      new PolygonRegion(
          new Translation2d[] {
            new Translation2d(4, 0),
            new Translation2d(4, Constants.DrivetrainConstants.CENTER_OFFSET_X),
            new Translation2d(8, Constants.DrivetrainConstants.CENTER_OFFSET_X),
            new Translation2d(8, 0),
            new Translation2d(4, 0),
          },
          "Processor Dead-zone");
  public static PolygonRegion BARGE_REGION =
      new PolygonRegion(
          new Translation2d[] {
            new Translation2d(
                (FieldConstants.FIELD_LENGTH / 2) - 0.737, FieldConstants.FIELD_WIDTH),
            new Translation2d((FieldConstants.FIELD_LENGTH / 2) - 3, FieldConstants.FIELD_WIDTH),
            new Translation2d(
                (FieldConstants.FIELD_LENGTH / 2) - 3, FieldConstants.FIELD_WIDTH / 2),
            new Translation2d(
                (FieldConstants.FIELD_LENGTH / 2) - 0.737, FieldConstants.FIELD_WIDTH / 2),
            new Translation2d((FieldConstants.FIELD_LENGTH / 2) - 0.737, FieldConstants.FIELD_WIDTH)
          },
          "Barge");
  public static PolygonRegion BARGE_ENTER =
      new PolygonRegion(
          new Translation2d[] {
            new Translation2d(
                (FieldConstants.FIELD_LENGTH / 2) - 0.737, FieldConstants.FIELD_WIDTH),
            new Translation2d((FieldConstants.FIELD_LENGTH / 2) - 3, FieldConstants.FIELD_WIDTH),
            new Translation2d(
                (FieldConstants.FIELD_LENGTH / 2) - 3, FieldConstants.FIELD_WIDTH / 2),
            new Translation2d(
                (FieldConstants.FIELD_LENGTH / 2) - 0.737, FieldConstants.FIELD_WIDTH / 2),
            new Translation2d((FieldConstants.FIELD_LENGTH / 2) - 0.737, FieldConstants.FIELD_WIDTH)
          },
          "BargeEnter");
  public static PolygonRegion RIGHT_CORAL_STATION_REGION =
      new PolygonRegion(
          new Translation2d[] {
            new Translation2d(0, 0),
            new Translation2d(0, 4),
            new Translation2d(4.5, 0),
            new Translation2d(0, 0)
          },
          "RightCoralStation");
  public static PolygonRegion RIGHT_CORAL_STATION_SLOW_REGION =
      new PolygonRegion(
          new Translation2d[] {
            // the corner to side of the coral station is 1.4m
            new Translation2d(0, 0),
            new Translation2d(0, 2.5),
            new Translation2d(2.9, 0),
            new Translation2d(0, 0)
          },
          "RightCoralStationSlow");
  public static PolygonRegion LEFT_CORAL_STATION_REGION =
      new PolygonRegion(
          new Translation2d[] {
            new Translation2d(0, FieldConstants.FIELD_WIDTH),
            new Translation2d(4.5, FieldConstants.FIELD_WIDTH),
            new Translation2d(0, FieldConstants.FIELD_WIDTH - 4),
            new Translation2d(0, FieldConstants.FIELD_WIDTH)
          },
          "LeftCoralStation");
  public static PolygonRegion LEFT_CORAL_STATION_SLOW_REGION =
      new PolygonRegion(
          new Translation2d[] {
            new Translation2d(0, FieldConstants.FIELD_WIDTH),
            new Translation2d(2.9, FieldConstants.FIELD_WIDTH),
            new Translation2d(0, FieldConstants.FIELD_WIDTH - 2.5),
            new Translation2d(0, FieldConstants.FIELD_WIDTH)
          },
          "LeftCoralStationSlow");
  public static PolygonRegion REEF_FACE0_REGION =
      new PolygonRegion(
          new Translation2d[] {
            FieldConstants.Reef.CENTER,
            // approximation of the right side of the right coral station
            new Translation2d(0, Units.inchesToMeters(50)),
            // approximation of the left side of the left coral station
            new Translation2d(0, FieldConstants.FIELD_WIDTH - Units.inchesToMeters(50)),
            FieldConstants.Reef.CENTER,
          },
          "ReefFace0");
  public static PolygonRegion REEF_FACE1_REGION =
      new PolygonRegion(
          new Translation2d[] {
            FieldConstants.Reef.CENTER,
            // approximation of the left side of the left coral station
            new Translation2d(0, FieldConstants.FIELD_WIDTH - Units.inchesToMeters(50)),
            // approximation of the right side of the left coral station
            new Translation2d(Units.inchesToMeters(68), FieldConstants.FIELD_WIDTH),
            // approximation of an imaginary line drawn from the left side of face1
            // to the left side of the field
            new Translation2d(Units.inchesToMeters(176), FieldConstants.FIELD_WIDTH),
            FieldConstants.Reef.CENTER,
          },
          "ReefFace1");
  public static PolygonRegion REEF_FACE2_REGION =
      new PolygonRegion(
          new Translation2d[] {
            FieldConstants.Reef.CENTER,
            // approximation of an imaginary line drawn from the right side of face2
            // to the left side of the field
            new Translation2d(Units.inchesToMeters(176), FieldConstants.FIELD_WIDTH),
            new Translation2d(FieldConstants.FIELD_LENGTH / 2, FieldConstants.FIELD_WIDTH),
            Barge.MIDDLE_CAGE,
            FieldConstants.Reef.CENTER,
          }, // FieldConstants.Reef.center cage
          // constant
          "ReefFace2");
  public static PolygonRegion REEF_FACE3_REGION =
      new PolygonRegion(
          new Translation2d[] {
            FieldConstants.Reef.CENTER,
            Barge.MIDDLE_CAGE,
            // approximation of opponents middle cage
            new Translation2d(FieldConstants.FIELD_LENGTH / 2, Units.inchesToMeters(56)),
            FieldConstants.Reef.CENTER,
          },
          "ReefFace3");
  public static PolygonRegion REEF_FACE4_REGION =
      new PolygonRegion(
          new Translation2d[] {
            FieldConstants.Reef.CENTER,
            // approximation of an imaginary line drawn from the left side of face4
            // to the right side of the field
            new Translation2d(Units.inchesToMeters(176), 0),
            new Translation2d(FieldConstants.FIELD_LENGTH / 2, 0),
            // approximation of the opponents FieldConstants.Reef.center cage
            new Translation2d(FieldConstants.FIELD_LENGTH / 2, Units.inchesToMeters(56)),
            FieldConstants.Reef.CENTER,
          },
          "ReefFace4");
  public static PolygonRegion REEF_FACE5_REGION =
      new PolygonRegion(
          new Translation2d[] {
            FieldConstants.Reef.CENTER,
            // approximation of an imaginary line drawn from the left side of the
            // face to the right side of the field
            new Translation2d(Units.inchesToMeters(176), 0),
            // approximation of the left side of the right coral station
            new Translation2d(Units.inchesToMeters(68), 0),
            // approximation of the right side of the right coral station
            new Translation2d(0, Units.inchesToMeters(50)),
            FieldConstants.Reef.CENTER,
          },
          "ReefFace5");

  // Region Lists
  public static PolygonRegion[] ALGAE_REGIONS = {BARGE_REGION, PROCESSOR_REGION};
  public static PolygonRegion[] STATION_REGIONS = {
    RIGHT_CORAL_STATION_REGION,
    LEFT_CORAL_STATION_REGION,
    RIGHT_CORAL_STATION_SLOW_REGION,
    LEFT_CORAL_STATION_SLOW_REGION
  };
  public static PolygonRegion[] REEF_REGIONS = {
    REEF_FACE0_REGION,
    REEF_FACE1_REGION,
    REEF_FACE2_REGION,
    REEF_FACE3_REGION,
    REEF_FACE4_REGION,
    REEF_FACE5_REGION
  };

  public static CircularRegion REEF_ENTER =
      new CircularRegion(FieldConstants.Reef.CENTER, 3, "Reef Enter");
  public static CircularRegion REEF_EXIT =
      new CircularRegion(FieldConstants.Reef.CENTER, 2, "Reef Exit");

  private static Region[] ALL_REGIONS = {
    BARGE_REGION,
    BARGE_ENTER,
    PROCESSOR_REGION,
    RIGHT_CORAL_STATION_REGION,
    LEFT_CORAL_STATION_REGION,
    RIGHT_CORAL_STATION_SLOW_REGION,
    LEFT_CORAL_STATION_SLOW_REGION,
    REEF_FACE0_REGION,
    REEF_FACE1_REGION,
    REEF_FACE2_REGION,
    REEF_FACE3_REGION,
    REEF_FACE4_REGION,
    REEF_FACE5_REGION,
    REEF_ENTER,
    REEF_EXIT
  };

  // Region to Target Pose Table
  // Poses are stored with a key equal to the name of the region
  public static Hashtable<String, Pose2d> REGION_POSE_TABLE = new Hashtable<>();

  public static void makeRegions() {
    for (Region region : ALL_REGIONS) {
      region.constructRegion();
    }
    populateTable();
  }

  /** Rotates all regions about the field center. */
  public static void flipRegions() {
    for (Region region : ALL_REGIONS) {
      region.allianceFlip();
    } 
    ScoringPoses.REEF_FACE_0_POSE = AllianceFlipUtil.apply(ScoringPoses.REEF_FACE_0_POSE);
    ScoringPoses.REEF_FACE_1_POSE = AllianceFlipUtil.apply(ScoringPoses.REEF_FACE_1_POSE);
    ScoringPoses.REEF_FACE_2_POSE = AllianceFlipUtil.apply(ScoringPoses.REEF_FACE_2_POSE);
    ScoringPoses.REEF_FACE_3_POSE = AllianceFlipUtil.apply(ScoringPoses.REEF_FACE_3_POSE);
    ScoringPoses.REEF_FACE_4_POSE = AllianceFlipUtil.apply(ScoringPoses.REEF_FACE_4_POSE);
    ScoringPoses.REEF_FACE_5_POSE = AllianceFlipUtil.apply(ScoringPoses.REEF_FACE_5_POSE);
    ScoringPoses.BARGE_TIGHT_ROPE.allianceFlip();
    ScoringPoses.PROCESSOR_TIGHT_ROPE.allianceFlip();

    populateTable();
  }

  private static void populateTable() {
    REGION_POSE_TABLE.put(
        RIGHT_CORAL_STATION_REGION.getName(), ScoringPoses.RIGHT_CORAL_STATION_POSE);
    REGION_POSE_TABLE.put(
        LEFT_CORAL_STATION_REGION.getName(), ScoringPoses.LEFT_CORAL_STATION_POSE);
    REGION_POSE_TABLE.put(REEF_FACE0_REGION.getName(), ScoringPoses.REEF_FACE_0_POSE);
    REGION_POSE_TABLE.put(REEF_FACE1_REGION.getName(), ScoringPoses.REEF_FACE_1_POSE);
    REGION_POSE_TABLE.put(REEF_FACE2_REGION.getName(), ScoringPoses.REEF_FACE_2_POSE);
    REGION_POSE_TABLE.put(REEF_FACE3_REGION.getName(), ScoringPoses.REEF_FACE_3_POSE);
    REGION_POSE_TABLE.put(REEF_FACE4_REGION.getName(), ScoringPoses.REEF_FACE_4_POSE);
    REGION_POSE_TABLE.put(REEF_FACE5_REGION.getName(), ScoringPoses.REEF_FACE_5_POSE);
  }
}
