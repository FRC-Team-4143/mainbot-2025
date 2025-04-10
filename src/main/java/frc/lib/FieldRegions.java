package frc.lib;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.lib.FieldConstants.Barge;
import frc.mw_lib.geometry.CircularRegion;
import frc.mw_lib.geometry.PolygonRegion;
import frc.mw_lib.geometry.Region;
import frc.robot.Constants;
import java.util.ArrayList;
import java.util.Hashtable;
import java.util.List;

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
          "PROCESSOR_REGION");
  public static PolygonRegion PROCESSOR_DEAD_REGION =
      new PolygonRegion(
          new Translation2d[] {
            new Translation2d(4, 0),
            new Translation2d(4, Constants.DrivetrainConstants.CENTER_OFFSET_X),
            new Translation2d(8, Constants.DrivetrainConstants.CENTER_OFFSET_X),
            new Translation2d(8, 0),
            new Translation2d(4, 0),
          },
          "PROCESSOR_DEAD_REGION");
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
          "BARGE_REGION");
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
          "BARGE_ENTER");
  public static PolygonRegion RIGHT_CORAL_STATION_REGION =
      new PolygonRegion(
          new Translation2d[] {
            new Translation2d(0, 0),
            new Translation2d(0, 4),
            new Translation2d(4.5, 0),
            new Translation2d(0, 0)
          },
          "RIGHT_CORAL_STATION_REGION");
  public static PolygonRegion RIGHT_CORAL_STATION_SLOW_REGION =
      new PolygonRegion(
          new Translation2d[] {
            // the corner to side of the coral station is 1.4m
            new Translation2d(0, 0),
            new Translation2d(0, 2.5),
            new Translation2d(2.9, 0),
            new Translation2d(0, 0)
          },
          "RIGHT_CORAL_STATION_SLOW_REGION");
  public static PolygonRegion LEFT_CORAL_STATION_REGION =
      new PolygonRegion(
          new Translation2d[] {
            new Translation2d(0, FieldConstants.FIELD_WIDTH),
            new Translation2d(4.5, FieldConstants.FIELD_WIDTH),
            new Translation2d(0, FieldConstants.FIELD_WIDTH - 4),
            new Translation2d(0, FieldConstants.FIELD_WIDTH)
          },
          "LEFT_CORAL_STATION_REGION");
  public static PolygonRegion LEFT_CORAL_STATION_SLOW_REGION =
      new PolygonRegion(
          new Translation2d[] {
            new Translation2d(0, FieldConstants.FIELD_WIDTH),
            new Translation2d(2.9, FieldConstants.FIELD_WIDTH),
            new Translation2d(0, FieldConstants.FIELD_WIDTH - 2.5),
            new Translation2d(0, FieldConstants.FIELD_WIDTH)
          },
          "LEFT_CORAL_STATION_SLOW_REGION");
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
          "REEF_FACE0_REGION");
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
          "REEF_FACE1_REGION");
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
          },
          "REEF_FACE2_REGION");
  public static PolygonRegion REEF_FACE3_REGION =
      new PolygonRegion(
          new Translation2d[] {
            FieldConstants.Reef.CENTER,
            Barge.MIDDLE_CAGE,
            // approximation of opponents middle cage
            new Translation2d(FieldConstants.FIELD_LENGTH / 2, Units.inchesToMeters(56)),
            FieldConstants.Reef.CENTER,
          },
          "REEF_FACE3_REGION");
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
          "REEF_FACE4_REGION");
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
          "REEF_FACE5_REGION");

  // Create Algae Stealing Regions
  public static PolygonRegion OPP_REEF_FACE0_REGION =
      new PolygonRegion(
          AllianceFlipUtil.apply(REEF_FACE0_REGION, FieldConstants.SYMMETRY_TYPE).getPoints(),
          "OPP_" + REEF_FACE0_REGION.getName());
  public static PolygonRegion OPP_REEF_FACE1_REGION =
      new PolygonRegion(
          AllianceFlipUtil.apply(REEF_FACE1_REGION, FieldConstants.SYMMETRY_TYPE).getPoints(),
          "OPP_" + REEF_FACE1_REGION.getName());
  public static PolygonRegion OPP_REEF_FACE2_REGION =
      new PolygonRegion(
          AllianceFlipUtil.apply(REEF_FACE2_REGION, FieldConstants.SYMMETRY_TYPE).getPoints(),
          "OPP_" + REEF_FACE2_REGION.getName());
  public static PolygonRegion OPP_REEF_FACE3_REGION =
      new PolygonRegion(
          AllianceFlipUtil.apply(REEF_FACE3_REGION, FieldConstants.SYMMETRY_TYPE).getPoints(),
          "OPP_" + REEF_FACE3_REGION.getName());
  public static PolygonRegion OPP_REEF_FACE4_REGION =
      new PolygonRegion(
          AllianceFlipUtil.apply(REEF_FACE4_REGION, FieldConstants.SYMMETRY_TYPE).getPoints(),
          "OPP_" + REEF_FACE4_REGION.getName());
  public static PolygonRegion OPP_REEF_FACE5_REGION =
      new PolygonRegion(
          AllianceFlipUtil.apply(REEF_FACE5_REGION, FieldConstants.SYMMETRY_TYPE).getPoints(),
          "OPP_" + REEF_FACE5_REGION.getName());

  public static PolygonRegion L4_COLLISION_REGION =
      new PolygonRegion(
          new Translation2d[] {
            new Translation2d(0, 0), new Translation2d(1, 0), new Translation2d(0, 1)
          },
          "L4_COLLISION_REGION");
  public static CircularRegion REEF_ENTER_REGION =
      new CircularRegion(FieldConstants.Reef.CENTER, 3, "REEF_ENTER_REGION");
  public static CircularRegion REEF_EXIT_REGION =
      new CircularRegion(FieldConstants.Reef.CENTER, 2, "REEF_EXIT_REGION");

  // Region Lists
  public static ArrayList<PolygonRegion> ALGAE_REGIONS =
      new ArrayList<>(List.of(BARGE_REGION, PROCESSOR_REGION));
  public static ArrayList<PolygonRegion> STATION_REGIONS =
      new ArrayList<>(
          List.of(
              RIGHT_CORAL_STATION_REGION,
              LEFT_CORAL_STATION_REGION,
              RIGHT_CORAL_STATION_SLOW_REGION,
              LEFT_CORAL_STATION_SLOW_REGION));
  public static ArrayList<PolygonRegion> REEF_REGIONS =
      new ArrayList<>(
          List.of(
              REEF_FACE0_REGION,
              REEF_FACE1_REGION,
              REEF_FACE2_REGION,
              REEF_FACE3_REGION,
              REEF_FACE4_REGION,
              REEF_FACE5_REGION,
              OPP_REEF_FACE0_REGION,
              OPP_REEF_FACE1_REGION,
              OPP_REEF_FACE2_REGION,
              OPP_REEF_FACE3_REGION,
              OPP_REEF_FACE4_REGION,
              OPP_REEF_FACE5_REGION));

  private static ArrayList<Region> ALL_REGIONS =
      new ArrayList<>(
          List.of(
              BARGE_REGION,
              BARGE_ENTER,
              PROCESSOR_REGION,
              PROCESSOR_DEAD_REGION,
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
              REEF_ENTER_REGION,
              REEF_EXIT_REGION,
              L4_COLLISION_REGION,
              OPP_REEF_FACE0_REGION,
              OPP_REEF_FACE1_REGION,
              OPP_REEF_FACE2_REGION,
              OPP_REEF_FACE3_REGION,
              OPP_REEF_FACE4_REGION,
              OPP_REEF_FACE5_REGION));

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
    for (int i = 0; i < ALL_REGIONS.size(); i++) {
      ALL_REGIONS.set(i, AllianceFlipUtil.apply(ALL_REGIONS.get(i), FieldConstants.SYMMETRY_TYPE));
    }

    // ScoringPoses.REEF_FACE_0_POSE =
    //     AllianceFlipUtil.apply(ScoringPoses.REEF_FACE_0_POSE, FieldConstants.SYMMETRY_TYPE);
    // ScoringPoses.REEF_FACE_1_POSE =
    //     AllianceFlipUtil.apply(ScoringPoses.REEF_FACE_1_POSE, FieldConstants.SYMMETRY_TYPE);
    // ScoringPoses.REEF_FACE_2_POSE =
    //     AllianceFlipUtil.apply(ScoringPoses.REEF_FACE_2_POSE, FieldConstants.SYMMETRY_TYPE);
    // ScoringPoses.REEF_FACE_3_POSE =
    //     AllianceFlipUtil.apply(ScoringPoses.REEF_FACE_3_POSE, FieldConstants.SYMMETRY_TYPE);
    // ScoringPoses.REEF_FACE_4_POSE =
    //     AllianceFlipUtil.apply(ScoringPoses.REEF_FACE_4_POSE, FieldConstants.SYMMETRY_TYPE);
    // ScoringPoses.REEF_FACE_5_POSE =
    //     AllianceFlipUtil.apply(ScoringPoses.REEF_FACE_5_POSE, FieldConstants.SYMMETRY_TYPE);

    // ScoringPoses.OPP_REEF_FACE_0_POSE =
    //     AllianceFlipUtil.apply(ScoringPoses.OPP_REEF_FACE_0_POSE, FieldConstants.SYMMETRY_TYPE);
    // ScoringPoses.OPP_REEF_FACE_1_POSE =
    //     AllianceFlipUtil.apply(ScoringPoses.OPP_REEF_FACE_1_POSE, FieldConstants.SYMMETRY_TYPE);
    // ScoringPoses.OPP_REEF_FACE_2_POSE =
    //     AllianceFlipUtil.apply(ScoringPoses.OPP_REEF_FACE_2_POSE, FieldConstants.SYMMETRY_TYPE);
    // ScoringPoses.OPP_REEF_FACE_3_POSE =
    //     AllianceFlipUtil.apply(ScoringPoses.OPP_REEF_FACE_3_POSE, FieldConstants.SYMMETRY_TYPE);
    // ScoringPoses.OPP_REEF_FACE_4_POSE =
    //     AllianceFlipUtil.apply(ScoringPoses.OPP_REEF_FACE_4_POSE, FieldConstants.SYMMETRY_TYPE);
    // ScoringPoses.OPP_REEF_FACE_5_POSE =
    //     AllianceFlipUtil.apply(ScoringPoses.OPP_REEF_FACE_5_POSE, FieldConstants.SYMMETRY_TYPE);

    ScoringPoses.BARGE_TIGHT_ROPE =
        AllianceFlipUtil.apply(ScoringPoses.BARGE_TIGHT_ROPE, FieldConstants.SYMMETRY_TYPE);
    ScoringPoses.PROCESSOR_TIGHT_ROPE =
        AllianceFlipUtil.apply(ScoringPoses.PROCESSOR_TIGHT_ROPE, FieldConstants.SYMMETRY_TYPE);

    System.out.println("Fliped all regions and tight ropes");

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

    REGION_POSE_TABLE.put(OPP_REEF_FACE0_REGION.getName(), ScoringPoses.OPP_REEF_FACE_0_POSE);
    REGION_POSE_TABLE.put(OPP_REEF_FACE1_REGION.getName(), ScoringPoses.OPP_REEF_FACE_1_POSE);
    REGION_POSE_TABLE.put(OPP_REEF_FACE2_REGION.getName(), ScoringPoses.OPP_REEF_FACE_2_POSE);
    REGION_POSE_TABLE.put(OPP_REEF_FACE3_REGION.getName(), ScoringPoses.OPP_REEF_FACE_3_POSE);
    REGION_POSE_TABLE.put(OPP_REEF_FACE4_REGION.getName(), ScoringPoses.OPP_REEF_FACE_4_POSE);
    REGION_POSE_TABLE.put(OPP_REEF_FACE5_REGION.getName(), ScoringPoses.OPP_REEF_FACE_5_POSE);

    System.out.println("populated Table");
  }
}
