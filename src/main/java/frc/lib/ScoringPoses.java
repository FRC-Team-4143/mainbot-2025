package frc.lib;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.mw_lib.geometry.TightRope;
import frc.mw_lib.util.ConstantsLoader;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.claw.ClawConstants;

public class ScoringPoses {

  private static final ConstantsLoader LOADER = ConstantsLoader.getInstance();
  public static final Transform2d L2_L3_OFFSET =
      new Transform2d(Units.inchesToMeters(-5), 0, new Rotation2d(0));

  // Offset to align robot bumpers with reef face
  public static final Transform2d REEF_FACE_OFFSET =
      new Transform2d(
          new Translation2d(DriveConstants.CENTER_OFFSET_X, 0), Rotation2d.fromDegrees(180));
  // Offset to align robot bumpers with coral station
  public static final Transform2d CORAL_STATION_OFFSET =
      new Transform2d(
          new Translation2d(DriveConstants.CENTER_OFFSET_X, 0), Rotation2d.fromDegrees(0));
  // Offset to align robot bumpers with processor face
  public static final Transform2d PROCESSOR_OFFSET =
      new Transform2d(
          new Translation2d(DriveConstants.CENTER_OFFSET_X, 0), Rotation2d.fromDegrees(180));
  // Offset to align robot bumpers with middle of barge
  public static final Transform2d BARGE_OFFSET =
      new Transform2d(
          new Translation2d(DriveConstants.CENTER_OFFSET_X, 0), Rotation2d.fromDegrees(180));

  public static final Transform2d ALGAE_ALIGN_OFFSET =
      new Transform2d(0, -ClawConstants.ALGAE_IMP_OFFSET, new Rotation2d());
  public static final double CORAL_ALIGN_OFFSET = -ClawConstants.CORAL_IMP_OFFSET;

  public static final Transform2d LEFT_COLUMN_OFFSET =
      new Transform2d(0, Units.inchesToMeters(6.47) + CORAL_ALIGN_OFFSET, new Rotation2d());
  public static final Transform2d RIGHT_COLUMN_OFFSET =
      new Transform2d(0, Units.inchesToMeters(-6.47) + CORAL_ALIGN_OFFSET, new Rotation2d());

  // Poses used for Scoring / Pickup Alignment
  public static Pose2d REEF_FACE_0_POSE =
      FieldConstants.Reef.CENTER_FACES[0].transformBy(REEF_FACE_OFFSET);
  public static Pose2d REEF_FACE_1_POSE =
      FieldConstants.Reef.CENTER_FACES[1].transformBy(REEF_FACE_OFFSET);
  public static Pose2d REEF_FACE_2_POSE =
      FieldConstants.Reef.CENTER_FACES[2].transformBy(REEF_FACE_OFFSET);
  public static Pose2d REEF_FACE_3_POSE =
      FieldConstants.Reef.CENTER_FACES[3].transformBy(REEF_FACE_OFFSET);
  public static Pose2d REEF_FACE_4_POSE =
      FieldConstants.Reef.CENTER_FACES[4].transformBy(REEF_FACE_OFFSET);
  public static Pose2d REEF_FACE_5_POSE =
      FieldConstants.Reef.CENTER_FACES[5].transformBy(REEF_FACE_OFFSET);

  // Poses used for Stealing Algae from Opposing Reef
  public static Pose2d OPP_REEF_FACE_0_POSE =
      FieldConstants.Reef.CENTER_FACES[0].transformBy(REEF_FACE_OFFSET);
  public static Pose2d OPP_REEF_FACE_1_POSE =
      FieldConstants.Reef.CENTER_FACES[1].transformBy(REEF_FACE_OFFSET);
  public static Pose2d OPP_REEF_FACE_2_POSE =
      FieldConstants.Reef.CENTER_FACES[2].transformBy(REEF_FACE_OFFSET);
  public static Pose2d OPP_REEF_FACE_3_POSE =
      FieldConstants.Reef.CENTER_FACES[3].transformBy(REEF_FACE_OFFSET);
  public static Pose2d OPP_REEF_FACE_4_POSE =
      FieldConstants.Reef.CENTER_FACES[4].transformBy(REEF_FACE_OFFSET);
  public static Pose2d OPP_REEF_FACE_5_POSE =
      FieldConstants.Reef.CENTER_FACES[5].transformBy(REEF_FACE_OFFSET);

  // Coral Station Poses
  public static Pose2d LEFT_CORAL_STATION_POSE =
      FieldConstants.CoralStation.LEFT_CENTER_FACE.transformBy(CORAL_STATION_OFFSET);
  public static Pose2d RIGHT_CORAL_STATION_POSE =
      FieldConstants.CoralStation.RIGHT_CENTER_FACE.transformBy(CORAL_STATION_OFFSET);
  // Barge Poses
  public static Pose2d BARGE_TIGHT_ROPE_POSE_A =
      new Pose2d(7.7562, FieldConstants.FIELD_WIDTH, Rotation2d.fromDegrees(0));
  public static Pose2d BARGE_TIGHT_ROPE_POSE_B =
      new Pose2d(
          7.7562,
          (FieldConstants.FIELD_WIDTH / 2)
              + Units.inchesToMeters(7)
              - ClawConstants.ALGAE_IMP_OFFSET
              + (FieldConstants.ALGAE_DIAMETER / 2),
          Rotation2d.fromDegrees(0));

  // Processor Poses
  public static Pose2d PROCESSOR_TIGHT_ROPE_POSE_A =
      FieldConstants.Processor.CENTER_FACE.transformBy(
          new Transform2d(
              3 - DriveConstants.CENTER_OFFSET_X,
              -ClawConstants.ALGAE_IMP_OFFSET,
              new Rotation2d()));
  public static Pose2d PROCESSOR_TIGHT_ROPE_POSE_B =
      FieldConstants.Processor.CENTER_FACE.transformBy(
          new Transform2d(0, -ClawConstants.ALGAE_IMP_OFFSET, new Rotation2d()));

  // Tight Ropes
  public static TightRope BARGE_TIGHT_ROPE =
      new TightRope(BARGE_TIGHT_ROPE_POSE_A, BARGE_TIGHT_ROPE_POSE_B, "Barge");
  public static TightRope OPP_BARGE_TIGHT_ROPE =
      new TightRope(BARGE_TIGHT_ROPE_POSE_A, BARGE_TIGHT_ROPE_POSE_B, "OPP_Barge");
  public static TightRope PROCESSOR_TIGHT_ROPE =
      new TightRope(PROCESSOR_TIGHT_ROPE_POSE_A, PROCESSOR_TIGHT_ROPE_POSE_B, "Processor");
}
