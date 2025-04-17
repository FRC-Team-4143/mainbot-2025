package frc.lib;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.AllianceFlipUtil.SymmetryType;
import frc.mw_lib.geometry.PolygonRegion;
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
  public static PolygonRegion OPP_BARGE_REGION =
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
          "OPP_BARGE_REGION");
  public static PolygonRegion OPP_BARGE_ENTER =
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
          "OPP_BARGE_ENTER");
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
  public static PolygonRegion L4_COLLISION_REGION =
      new PolygonRegion(
          new Translation2d[] {
            new Translation2d(3.102, 4.827),
            new Translation2d(4.489, 5.628),
            new Translation2d(5.877, 4.827),
            new Translation2d(5.877, 3.225),
            new Translation2d(4.489, 2.424),
            new Translation2d(3.102, 3.225),
            new Translation2d(3.102, 4.827)
          },
          "L4_COLLISION_REGION");

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
            new Translation2d(
                FieldConstants.FIELD_LENGTH / 2,
                FieldConstants.FIELD_WIDTH - Units.inchesToMeters(56)),
            FieldConstants.Reef.CENTER,
          },
          "REEF_FACE2_REGION");
  public static PolygonRegion REEF_FACE3_REGION =
      new PolygonRegion(
          new Translation2d[] {
            FieldConstants.Reef.CENTER,
            new Translation2d(
                FieldConstants.FIELD_LENGTH / 2,
                FieldConstants.FIELD_WIDTH - Units.inchesToMeters(56)),
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

  public static PolygonRegion OPP_REEF_FACE0_REGION =
      new PolygonRegion(
          new Translation2d[] {
            FieldConstants.Reef.CENTER,
            // approximation of the right side of the right coral station
            new Translation2d(0, Units.inchesToMeters(50)),
            // approximation of the left side of the left coral station
            new Translation2d(0, FieldConstants.FIELD_WIDTH - Units.inchesToMeters(50)),
            FieldConstants.Reef.CENTER,
          },
          "OPP_REEF_FACE0_REGION");
  public static PolygonRegion OPP_REEF_FACE1_REGION =
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
          "OPP_REEF_FACE1_REGION");
  public static PolygonRegion OPP_REEF_FACE2_REGION =
      new PolygonRegion(
          new Translation2d[] {
            FieldConstants.Reef.CENTER,
            // approximation of an imaginary line drawn from the right side of face2
            // to the left side of the field
            new Translation2d(Units.inchesToMeters(176), FieldConstants.FIELD_WIDTH),
            new Translation2d(FieldConstants.FIELD_LENGTH / 2, FieldConstants.FIELD_WIDTH),
            new Translation2d(
                FieldConstants.FIELD_LENGTH / 2,
                FieldConstants.FIELD_WIDTH - Units.inchesToMeters(56)),
            FieldConstants.Reef.CENTER,
          },
          "OPP_REEF_FACE2_REGION");
  public static PolygonRegion OPP_REEF_FACE3_REGION =
      new PolygonRegion(
          new Translation2d[] {
            FieldConstants.Reef.CENTER,
            new Translation2d(
                FieldConstants.FIELD_LENGTH / 2,
                FieldConstants.FIELD_WIDTH - Units.inchesToMeters(56)),
            // approximation of opponents middle cage
            new Translation2d(FieldConstants.FIELD_LENGTH / 2, Units.inchesToMeters(56)),
            FieldConstants.Reef.CENTER,
          },
          "OPP_REEF_FACE3_REGION");
  public static PolygonRegion OPP_REEF_FACE4_REGION =
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
          "OPP_REEF_FACE4_REGION");
  public static PolygonRegion OPP_REEF_FACE5_REGION =
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
          "OPP_REEF_FACE5_REGION");

  public static PolygonRegion REEF_ENTER_REGION =
      new PolygonRegion(
          new Translation2d[] {
            new Translation2d(4.489, 0.562),
            new Translation2d(7.489, 2.294),
            new Translation2d(7.489, 5.758),
            new Translation2d(4.489, 7.49),
            new Translation2d(1.489, 5.758),
            new Translation2d(1.489, 2.294),
            new Translation2d(4.489, 0.562)
          },
          "REEF_ENTER_REGION");
  public static PolygonRegion REEF_EXIT_REGION =
      new PolygonRegion(
          new Translation2d[] {
            new Translation2d(4.489, 1.717),
            new Translation2d(6.489, 2.871),
            new Translation2d(6.489, 5.181),
            new Translation2d(4.489, 6.335),
            new Translation2d(2.489, 5.181),
            new Translation2d(2.489, 2.871),
            new Translation2d(4.489, 1.717)
          },
          "REEF_EXIT_REGION");

  public static PolygonRegion OPP_REEF_ENTER_REGION =
      new PolygonRegion(
          new Translation2d[] {
            new Translation2d(4.489, 0.562),
            new Translation2d(7.489, 2.294),
            new Translation2d(7.489, 5.758),
            new Translation2d(4.489, 7.49),
            new Translation2d(1.489, 5.758),
            new Translation2d(1.489, 2.294),
            new Translation2d(4.489, 0.562)
          },
          "OPP_REEF_ENTER_REGION");
  public static PolygonRegion OPP_REEF_EXIT_REGION =
      new PolygonRegion(
          new Translation2d[] {
            new Translation2d(4.489, 1.717),
            new Translation2d(6.489, 2.871),
            new Translation2d(6.489, 5.181),
            new Translation2d(4.489, 6.335),
            new Translation2d(2.489, 5.181),
            new Translation2d(2.489, 2.871),
            new Translation2d(4.489, 1.717)
          },
          "OPP_REEF_EXIT_REGION");

  // Region List
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

  // Region to Target Pose Table
  // Poses are stored with a key equal to the name of the region
  public static Hashtable<String, Pose2d> REGION_POSE_TABLE = new Hashtable<>();

  public static void makeRegions() {
    OPP_REEF_FACE0_REGION =
        AllianceFlipUtil.apply(OPP_REEF_FACE0_REGION, FieldConstants.SYMMETRY_TYPE);
    OPP_REEF_FACE1_REGION =
        AllianceFlipUtil.apply(OPP_REEF_FACE1_REGION, FieldConstants.SYMMETRY_TYPE);
    OPP_REEF_FACE2_REGION =
        AllianceFlipUtil.apply(OPP_REEF_FACE2_REGION, FieldConstants.SYMMETRY_TYPE);
    OPP_REEF_FACE3_REGION =
        AllianceFlipUtil.apply(OPP_REEF_FACE3_REGION, FieldConstants.SYMMETRY_TYPE);
    OPP_REEF_FACE4_REGION =
        AllianceFlipUtil.apply(OPP_REEF_FACE4_REGION, FieldConstants.SYMMETRY_TYPE);
    OPP_REEF_FACE4_REGION =
        AllianceFlipUtil.apply(OPP_REEF_FACE5_REGION, FieldConstants.SYMMETRY_TYPE);
    OPP_REEF_ENTER_REGION =
        AllianceFlipUtil.apply(OPP_REEF_ENTER_REGION, FieldConstants.SYMMETRY_TYPE);
    OPP_REEF_EXIT_REGION =
        AllianceFlipUtil.apply(OPP_REEF_EXIT_REGION, FieldConstants.SYMMETRY_TYPE);

    // Opposing Alliance Reef Regions start as Current Alliance and need flipped on construction
    ScoringPoses.OPP_REEF_FACE_0_POSE =
        AllianceFlipUtil.apply(ScoringPoses.OPP_REEF_FACE_0_POSE, FieldConstants.SYMMETRY_TYPE);
    ScoringPoses.OPP_REEF_FACE_1_POSE =
        AllianceFlipUtil.apply(ScoringPoses.OPP_REEF_FACE_1_POSE, FieldConstants.SYMMETRY_TYPE);
    ScoringPoses.OPP_REEF_FACE_2_POSE =
        AllianceFlipUtil.apply(ScoringPoses.OPP_REEF_FACE_2_POSE, FieldConstants.SYMMETRY_TYPE);
    ScoringPoses.OPP_REEF_FACE_3_POSE =
        AllianceFlipUtil.apply(ScoringPoses.OPP_REEF_FACE_3_POSE, FieldConstants.SYMMETRY_TYPE);
    ScoringPoses.OPP_REEF_FACE_4_POSE =
        AllianceFlipUtil.apply(ScoringPoses.OPP_REEF_FACE_4_POSE, FieldConstants.SYMMETRY_TYPE);
    ScoringPoses.OPP_REEF_FACE_5_POSE =
        AllianceFlipUtil.apply(ScoringPoses.OPP_REEF_FACE_5_POSE, FieldConstants.SYMMETRY_TYPE);

    OPP_BARGE_REGION = AllianceFlipUtil.apply(OPP_BARGE_REGION, SymmetryType.DIRECT);
    OPP_BARGE_ENTER = AllianceFlipUtil.apply(OPP_BARGE_ENTER, SymmetryType.DIRECT);
    ScoringPoses.OPP_BARGE_TIGHT_ROPE =
        AllianceFlipUtil.apply(ScoringPoses.OPP_BARGE_TIGHT_ROPE, SymmetryType.DIRECT);

    populateTable();
  }

  /** Rotates all regions about the field center. */
  public static void flipRegions() {
    DataLogManager.log("Flipping Regions to " + DriverStation.getAlliance().get().toString());

    // Current Alliance Reef Regions
    REEF_FACE0_REGION = AllianceFlipUtil.apply(REEF_FACE0_REGION, FieldConstants.SYMMETRY_TYPE);
    REEF_FACE1_REGION = AllianceFlipUtil.apply(REEF_FACE1_REGION, FieldConstants.SYMMETRY_TYPE);
    REEF_FACE2_REGION = AllianceFlipUtil.apply(REEF_FACE2_REGION, FieldConstants.SYMMETRY_TYPE);
    REEF_FACE3_REGION = AllianceFlipUtil.apply(REEF_FACE3_REGION, FieldConstants.SYMMETRY_TYPE);
    REEF_FACE4_REGION = AllianceFlipUtil.apply(REEF_FACE4_REGION, FieldConstants.SYMMETRY_TYPE);
    REEF_FACE4_REGION = AllianceFlipUtil.apply(REEF_FACE5_REGION, FieldConstants.SYMMETRY_TYPE);
    REEF_ENTER_REGION = AllianceFlipUtil.apply(REEF_ENTER_REGION, FieldConstants.SYMMETRY_TYPE);
    REEF_EXIT_REGION = AllianceFlipUtil.apply(REEF_EXIT_REGION, FieldConstants.SYMMETRY_TYPE);
    L4_COLLISION_REGION = AllianceFlipUtil.apply(L4_COLLISION_REGION, FieldConstants.SYMMETRY_TYPE);

    ScoringPoses.REEF_FACE_0_POSE =
        AllianceFlipUtil.apply(ScoringPoses.REEF_FACE_0_POSE, FieldConstants.SYMMETRY_TYPE);
    ScoringPoses.REEF_FACE_1_POSE =
        AllianceFlipUtil.apply(ScoringPoses.REEF_FACE_1_POSE, FieldConstants.SYMMETRY_TYPE);
    ScoringPoses.REEF_FACE_2_POSE =
        AllianceFlipUtil.apply(ScoringPoses.REEF_FACE_2_POSE, FieldConstants.SYMMETRY_TYPE);
    ScoringPoses.REEF_FACE_3_POSE =
        AllianceFlipUtil.apply(ScoringPoses.REEF_FACE_3_POSE, FieldConstants.SYMMETRY_TYPE);
    ScoringPoses.REEF_FACE_4_POSE =
        AllianceFlipUtil.apply(ScoringPoses.REEF_FACE_4_POSE, FieldConstants.SYMMETRY_TYPE);
    ScoringPoses.REEF_FACE_5_POSE =
        AllianceFlipUtil.apply(ScoringPoses.REEF_FACE_5_POSE, FieldConstants.SYMMETRY_TYPE);

    // Opposing Alliance Reef Regions
    OPP_REEF_FACE0_REGION =
        AllianceFlipUtil.apply(OPP_REEF_FACE0_REGION, FieldConstants.SYMMETRY_TYPE);
    OPP_REEF_FACE1_REGION =
        AllianceFlipUtil.apply(OPP_REEF_FACE1_REGION, FieldConstants.SYMMETRY_TYPE);
    OPP_REEF_FACE2_REGION =
        AllianceFlipUtil.apply(OPP_REEF_FACE2_REGION, FieldConstants.SYMMETRY_TYPE);
    OPP_REEF_FACE3_REGION =
        AllianceFlipUtil.apply(OPP_REEF_FACE3_REGION, FieldConstants.SYMMETRY_TYPE);
    OPP_REEF_FACE4_REGION =
        AllianceFlipUtil.apply(OPP_REEF_FACE4_REGION, FieldConstants.SYMMETRY_TYPE);
    OPP_REEF_FACE5_REGION =
        AllianceFlipUtil.apply(OPP_REEF_FACE5_REGION, FieldConstants.SYMMETRY_TYPE);
    OPP_REEF_ENTER_REGION =
        AllianceFlipUtil.apply(OPP_REEF_ENTER_REGION, FieldConstants.SYMMETRY_TYPE);
    OPP_REEF_EXIT_REGION =
        AllianceFlipUtil.apply(OPP_REEF_EXIT_REGION, FieldConstants.SYMMETRY_TYPE);

    ScoringPoses.OPP_REEF_FACE_0_POSE =
        AllianceFlipUtil.apply(ScoringPoses.OPP_REEF_FACE_0_POSE, FieldConstants.SYMMETRY_TYPE);
    ScoringPoses.OPP_REEF_FACE_1_POSE =
        AllianceFlipUtil.apply(ScoringPoses.OPP_REEF_FACE_1_POSE, FieldConstants.SYMMETRY_TYPE);
    ScoringPoses.OPP_REEF_FACE_2_POSE =
        AllianceFlipUtil.apply(ScoringPoses.OPP_REEF_FACE_2_POSE, FieldConstants.SYMMETRY_TYPE);
    ScoringPoses.OPP_REEF_FACE_3_POSE =
        AllianceFlipUtil.apply(ScoringPoses.OPP_REEF_FACE_3_POSE, FieldConstants.SYMMETRY_TYPE);
    ScoringPoses.OPP_REEF_FACE_4_POSE =
        AllianceFlipUtil.apply(ScoringPoses.OPP_REEF_FACE_4_POSE, FieldConstants.SYMMETRY_TYPE);
    ScoringPoses.OPP_REEF_FACE_5_POSE =
        AllianceFlipUtil.apply(ScoringPoses.OPP_REEF_FACE_5_POSE, FieldConstants.SYMMETRY_TYPE);

    // Update REEF_REGIONS list for zone checking
    REEF_REGIONS.add(0, REEF_FACE0_REGION);
    REEF_REGIONS.add(1, REEF_FACE1_REGION);
    REEF_REGIONS.add(2, REEF_FACE2_REGION);
    REEF_REGIONS.add(3, REEF_FACE3_REGION);
    REEF_REGIONS.add(4, REEF_FACE4_REGION);
    REEF_REGIONS.add(5, REEF_FACE5_REGION);
    REEF_REGIONS.add(6, OPP_REEF_FACE0_REGION);
    REEF_REGIONS.add(7, OPP_REEF_FACE1_REGION);
    REEF_REGIONS.add(8, OPP_REEF_FACE2_REGION);
    REEF_REGIONS.add(9, OPP_REEF_FACE3_REGION);
    REEF_REGIONS.add(10, OPP_REEF_FACE4_REGION);
    REEF_REGIONS.add(11, OPP_REEF_FACE5_REGION);

    // Algae Poses
    BARGE_REGION = AllianceFlipUtil.apply(BARGE_REGION, FieldConstants.SYMMETRY_TYPE);
    BARGE_ENTER = AllianceFlipUtil.apply(BARGE_ENTER, FieldConstants.SYMMETRY_TYPE);
    OPP_BARGE_REGION = AllianceFlipUtil.apply(OPP_BARGE_REGION, FieldConstants.SYMMETRY_TYPE);
    OPP_BARGE_ENTER = AllianceFlipUtil.apply(OPP_BARGE_ENTER, FieldConstants.SYMMETRY_TYPE);
    PROCESSOR_REGION = AllianceFlipUtil.apply(PROCESSOR_REGION, FieldConstants.SYMMETRY_TYPE);
    PROCESSOR_DEAD_REGION =
        AllianceFlipUtil.apply(PROCESSOR_DEAD_REGION, FieldConstants.SYMMETRY_TYPE);

    ScoringPoses.BARGE_TIGHT_ROPE =
        AllianceFlipUtil.apply(ScoringPoses.BARGE_TIGHT_ROPE, FieldConstants.SYMMETRY_TYPE);
    ScoringPoses.OPP_BARGE_TIGHT_ROPE =
        AllianceFlipUtil.apply(ScoringPoses.OPP_BARGE_TIGHT_ROPE, FieldConstants.SYMMETRY_TYPE);
    ScoringPoses.PROCESSOR_TIGHT_ROPE =
        AllianceFlipUtil.apply(ScoringPoses.PROCESSOR_TIGHT_ROPE, FieldConstants.SYMMETRY_TYPE);

    // Station Regions
    RIGHT_CORAL_STATION_REGION =
        AllianceFlipUtil.apply(RIGHT_CORAL_STATION_REGION, FieldConstants.SYMMETRY_TYPE);
    RIGHT_CORAL_STATION_SLOW_REGION =
        AllianceFlipUtil.apply(RIGHT_CORAL_STATION_SLOW_REGION, FieldConstants.SYMMETRY_TYPE);
    LEFT_CORAL_STATION_REGION =
        AllianceFlipUtil.apply(LEFT_CORAL_STATION_REGION, FieldConstants.SYMMETRY_TYPE);
    LEFT_CORAL_STATION_SLOW_REGION =
        AllianceFlipUtil.apply(LEFT_CORAL_STATION_SLOW_REGION, FieldConstants.SYMMETRY_TYPE);

    populateTable();
  }

  private static void populateTable() {
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
  }
}
