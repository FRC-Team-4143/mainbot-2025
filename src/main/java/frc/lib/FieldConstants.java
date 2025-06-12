// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.lib;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import frc.lib.AllianceFlipUtil.SymmetryType;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Contains various field dimensions and useful reference points. All units are in meters and poses
 * have a blue alliance origin.
 */
public class FieldConstants {
  public static final double FIELD_LENGTH = Units.inchesToMeters(690.876);
  public static final double FIELD_WIDTH = Units.inchesToMeters(317);
  public static final SymmetryType SYMMETRY_TYPE = SymmetryType.DIAGONAL;
  public static final Translation2d FIELD_CENTER =
      new Translation2d(FIELD_LENGTH / 2, FIELD_WIDTH / 2);
  public static final double STARTING_LINE_X =
      Units.inchesToMeters(299.438); // Measured from the inside of starting line
  public static final double ALGAE_DIAMETER = Units.inchesToMeters(16);

  public static class Processor {
    public static final Pose2d CENTER_FACE =
        new Pose2d(Units.inchesToMeters(235.726), 0, Rotation2d.fromDegrees(90));
  }

  public static class Barge {
    public static final Translation2d FAR_CAGE =
        new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(286.779));
    public static final Translation2d MIDDLE_CAGE =
        new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(242.855));
    public static final Translation2d CLOSE_CAGE =
        new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(199.947));

    // Measured from floor to bottom of cage
    public static final double DEEP_HEIGHT = Units.inchesToMeters(3.125);
    public static final double SHALLOW_HEIGHT = Units.inchesToMeters(30.125);
  }

  public static class CoralStation {
    public static final Pose2d LEFT_CENTER_FACE =
        new Pose2d(
            Units.inchesToMeters(33.526),
            Units.inchesToMeters(291.176),
            Rotation2d.fromDegrees(90 - 144.011));
    public static final Pose2d RIGHT_CENTER_FACE =
        new Pose2d(
            Units.inchesToMeters(33.526),
            Units.inchesToMeters(25.824),
            Rotation2d.fromDegrees(144.011 - 90));
  }

  public static class Reef {
    public static final Translation2d CENTER =
        new Translation2d(Units.inchesToMeters(176.746), Units.inchesToMeters(158.501));
    public static final double FACE_TO_ZONE_LINE =
        Units.inchesToMeters(12); // Side of the reef to the inside of the reef zone line

    public static final Pose2d[] CENTER_FACES =
        new Pose2d[6]; // Starting facing the driver station in clockwise order
    public static final List<Map<ReefHeight, Pose3d>> BRANCH_POSITIONS =
        new ArrayList<>(); // Starting at the right branch facing the driver station in clockwise

    static {
      // Initialize faces
      CENTER_FACES[0] =
          new Pose2d(
              Units.inchesToMeters(144.003),
              Units.inchesToMeters(158.500),
              Rotation2d.fromDegrees(180));
      CENTER_FACES[1] =
          new Pose2d(
              Units.inchesToMeters(160.373),
              Units.inchesToMeters(186.857),
              Rotation2d.fromDegrees(120));
      CENTER_FACES[2] =
          new Pose2d(
              Units.inchesToMeters(193.116),
              Units.inchesToMeters(186.858),
              Rotation2d.fromDegrees(60));
      CENTER_FACES[3] =
          new Pose2d(
              Units.inchesToMeters(209.489),
              Units.inchesToMeters(158.502),
              Rotation2d.fromDegrees(0));
      CENTER_FACES[4] =
          new Pose2d(
              Units.inchesToMeters(193.118),
              Units.inchesToMeters(130.145),
              Rotation2d.fromDegrees(-60));
      CENTER_FACES[5] =
          new Pose2d(
              Units.inchesToMeters(160.375),
              Units.inchesToMeters(130.144),
              Rotation2d.fromDegrees(-120));

      // Initialize branch positions
      for (int face = 0; face < 6; face++) {
        Map<ReefHeight, Pose3d> fillRight = new HashMap<>();
        Map<ReefHeight, Pose3d> fillLeft = new HashMap<>();
        for (var level : ReefHeight.values()) {
          Pose2d poseDirection = new Pose2d(CENTER, Rotation2d.fromDegrees(180 - (60 * face)));
          double adjustX = Units.inchesToMeters(30.738);
          double adjustY = Units.inchesToMeters(6.469);

          fillRight.put(
              level,
              new Pose3d(
                  new Translation3d(
                      poseDirection
                          .transformBy(new Transform2d(adjustX, adjustY, new Rotation2d()))
                          .getX(),
                      poseDirection
                          .transformBy(new Transform2d(adjustX, adjustY, new Rotation2d()))
                          .getY(),
                      level.HEIGHT),
                  new Rotation3d(
                      0, (level.PITCH.getRadians()), poseDirection.getRotation().getRadians())));
          fillLeft.put(
              level,
              new Pose3d(
                  new Translation3d(
                      poseDirection
                          .transformBy(new Transform2d(adjustX, -adjustY, new Rotation2d()))
                          .getX(),
                      poseDirection
                          .transformBy(new Transform2d(adjustX, -adjustY, new Rotation2d()))
                          .getY(),
                      level.HEIGHT),
                  new Rotation3d(
                      0, (level.PITCH.getRadians()), poseDirection.getRotation().getRadians())));
        }
        BRANCH_POSITIONS.add(fillRight);
        BRANCH_POSITIONS.add(fillLeft);
      }
    }
  }

  public static class StagingPositions {
    // Measured from the center of the ice cream
    public static final Pose2d LEFT_ICE_CREAM =
        new Pose2d(Units.inchesToMeters(48), Units.inchesToMeters(230.5), new Rotation2d());
    public static final Pose2d MIDDLE_ICE_CREAM =
        new Pose2d(Units.inchesToMeters(48), Units.inchesToMeters(158.5), new Rotation2d());
    public static final Pose2d RIGHT_ICE_CREAM =
        new Pose2d(Units.inchesToMeters(48), Units.inchesToMeters(86.5), new Rotation2d());
  }

  public enum ReefHeight {
    L4(3, Units.inchesToMeters(72), Rotation2d.fromDegrees(-90)),
    L3(2, Units.inchesToMeters(47.625), Rotation2d.fromDegrees(-35)),
    L2(1, Units.inchesToMeters(31.875), Rotation2d.fromDegrees(-35)),
    L1(0, Units.inchesToMeters(18), Rotation2d.fromDegrees(0));

    ReefHeight(int level_number, double height, Rotation2d pitch) {
      this.LEVEL_NUMBER = level_number;
      this.HEIGHT = height;
      this.PITCH = pitch; // in degrees
    }

    public static ReefHeight fromLevel(int level) {
      return Arrays.stream(values())
          .filter(height -> height.ordinal() == level)
          .findFirst()
          .orElse(L4);
    }

    public final int LEVEL_NUMBER;
    public final double HEIGHT;
    public final Rotation2d PITCH;
  }
}
