package frc.lib;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.lib.FieldConstants.Barge;
import frc.mw_lib.geometry.CircularRegion;
import frc.mw_lib.geometry.PolygonRegion;

public class FieldRegions {

  public static PolygonRegion PROCESSOR_REGION =
      new PolygonRegion(
          new Translation2d[] {
            new Translation2d(4, 0),
            new Translation2d(4, 2),
            new Translation2d(8, 2),
            new Translation2d(8, 0),
            new Translation2d(4, 0),
          },
          "Processor");

  public static PolygonRegion BARGE_REGION =
      new PolygonRegion(
          new Translation2d[] {
            new Translation2d(FieldConstants.FIELD_LENGTH / 2, FieldConstants.FIELD_WIDTH),
            new Translation2d((FieldConstants.FIELD_LENGTH / 2) - 2, FieldConstants.FIELD_WIDTH),
            new Translation2d(
                (FieldConstants.FIELD_LENGTH / 2) - 2, FieldConstants.FIELD_WIDTH / 2),
            new Translation2d(FieldConstants.FIELD_LENGTH / 2, FieldConstants.FIELD_WIDTH / 2)
          },
          "Barge");
  public static PolygonRegion RIGHT_CORAL_STATION_REGION =
      new PolygonRegion(
          new Translation2d[] {
            new Translation2d(0, 0),
            new Translation2d(0, FieldConstants.FIELD_WIDTH / 2),
            new Translation2d(4, FieldConstants.FIELD_WIDTH / 2),
            new Translation2d(4, 0)
          },
          "RightCoralStation");

  public static PolygonRegion LEFT_CORAL_STATION_REGION =
      new PolygonRegion(
          new Translation2d[] {
            new Translation2d(0, FieldConstants.FIELD_WIDTH),
            new Translation2d(4, FieldConstants.FIELD_WIDTH),
            new Translation2d(4, FieldConstants.FIELD_WIDTH / 2),
            new Translation2d(0, FieldConstants.FIELD_WIDTH / 2)
          },
          "LeftCoralStation");

  // add regions
  public static PolygonRegion REEF_FACE0_REGION =
      new PolygonRegion(
          new Translation2d[] {
            FieldConstants.Reef.CENTER,
            // approximation of the right side of the right coral station
            new Translation2d(0, Units.feetToMeters(4)),
            // approximation of the left side of the left coral station
            new Translation2d(0, FieldConstants.FIELD_WIDTH - Units.feetToMeters(4)),
            FieldConstants.Reef.CENTER,
          },
          "ReefFace0");

  public static PolygonRegion REEF_FACE1_REGION =
      new PolygonRegion(
          new Translation2d[] {
            FieldConstants.Reef.CENTER,
            // approximation of the left side of the left coral station
            new Translation2d(0, FieldConstants.FIELD_WIDTH - Units.feetToMeters(4)),
            // approximation of the right side of the left coral station
            new Translation2d(Units.feetToMeters(6), FieldConstants.FIELD_WIDTH),
            // approximation of an imaginary line drawn from the left side of face1
            // to the left side of the field
            new Translation2d(
                Units.feetToMeters(4) + Units.feetToMeters(10), FieldConstants.FIELD_WIDTH),
            FieldConstants.Reef.CENTER,
          },
          "ReefFace1");

  public static PolygonRegion REEF_FACE2_REGION =
      new PolygonRegion(
          new Translation2d[] {
            FieldConstants.Reef.CENTER,
            // approximation of an imaginary line drawn from the right side of face2
            // to the left side of the field
            new Translation2d(
                Units.feetToMeters(4) + Units.feetToMeters(10), FieldConstants.FIELD_WIDTH),
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
            new Translation2d(Units.feetToMeters(4) + Units.feetToMeters(10), 0),
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
            new Translation2d(Units.feetToMeters(4) + Units.feetToMeters(10), 0),
            // approximation of the left side of the right coral station
            new Translation2d(Units.feetToMeters(6), 0),
            // approximation of the right side of the right coral station
            new Translation2d(0, Units.feetToMeters(4)),
            FieldConstants.Reef.CENTER,
          },
          "ReefFace5");

  // region lists
  public static PolygonRegion[] ALGAE_REGIONS = {BARGE_REGION, PROCESSOR_REGION};
  public static PolygonRegion[] STATION_REGIONS = {
    RIGHT_CORAL_STATION_REGION, LEFT_CORAL_STATION_REGION
  };
  public static PolygonRegion[] REEF_REGIONS = {
    REEF_FACE0_REGION,
    REEF_FACE1_REGION,
    REEF_FACE2_REGION,
    REEF_FACE3_REGION,
    REEF_FACE4_REGION,
    REEF_FACE0_REGION,
    REEF_FACE5_REGION
  };

  public static CircularRegion REEF_ENTER =
      new CircularRegion(FieldConstants.Reef.CENTER, 2, "reef_enter");
  public static CircularRegion REEF_EXIT =
      new CircularRegion(FieldConstants.Reef.CENTER, 3, "reef_exit");

  public static void constructRegions(boolean flip) {
    for (PolygonRegion region : ALGAE_REGIONS) {
      region.constructAllianceRegion(flip);
    }
    for (PolygonRegion region : REEF_REGIONS) {
      region.constructAllianceRegion(flip);
    }
    for (PolygonRegion region : STATION_REGIONS) {
      region.constructAllianceRegion(flip);
    }
    REEF_ENTER.constructAllianceRegion(flip);
    REEF_EXIT.constructAllianceRegion(flip);
  }
}
