package frc.lib;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.mw_lib.util.ConstantsLoader;
import frc.robot.Constants.DrivetrainConstants;

public class ScoringPoses {

  private static final ConstantsLoader LOADER = ConstantsLoader.getInstance();

  // Offset to aligin robot bumpers with reef face
  public static final Transform2d REEF_FACE_OFFSET =
      new Transform2d(
          new Translation2d(DrivetrainConstants.CENTER_OFFSET_X, 0), Rotation2d.fromDegrees(180));
  // Offset to aligin robot bumpers with coral station
  public static final Transform2d CORAL_STATION_OFFSET =
      new Transform2d(
          new Translation2d(DrivetrainConstants.CENTER_OFFSET_X, 0), Rotation2d.fromDegrees(180));
  // Offset to aligin robot bumpers with processor face
  public static final Transform2d PROCESSOR_OFFSET =
      new Transform2d(
          new Translation2d(DrivetrainConstants.CENTER_OFFSET_X, 0), Rotation2d.fromDegrees(180));
  // Offset to aligin robot bumpers with middle of barge
  public static final Transform2d BARGE_OFFSET =
      new Transform2d(
          new Translation2d(DrivetrainConstants.CENTER_OFFSET_X, 0), Rotation2d.fromDegrees(180));

  public static final Transform2d ALGAE_ALIGN_OFFSET =
      new Transform2d(0, LOADER.getDoubleValue("imp", "algae_offset"), new Rotation2d());
  public static final Transform2d CORAL_ALIGN_OFFSET =
      new Transform2d(0, LOADER.getDoubleValue("imp", "coral_offset"), new Rotation2d());

  public static final Transform2d LEFT_COLUMN_OFFEST =
      new Transform2d(0, Units.inchesToMeters(6.47), new Rotation2d()).plus(CORAL_ALIGN_OFFSET);
  public static final Transform2d RIGHT_COLUMN_OFFSET =
      new Transform2d(0, Units.inchesToMeters(6.47), new Rotation2d()).plus(CORAL_ALIGN_OFFSET);

  // Poses used for scoring / pickup alignment
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
  public static Pose2d LEFT_CORAL_STATION_POSE =
      FieldConstants.CoralStation.LEFT_CENTER_FACE.transformBy(CORAL_STATION_OFFSET);
  public static Pose2d RIGHT_CORAL_STATION_POSE =
      FieldConstants.CoralStation.RIGHT_CENTER_FACE.transformBy(CORAL_STATION_OFFSET);
  public static Pose2d PROCESSOR_POSE =
      FieldConstants.Processor.CENTER_FACE.transformBy(PROCESSOR_OFFSET);
  public static Pose2d BARGE_POSE =
      new Pose2d(FieldConstants.Barge.MIDDLE_CAGE, Rotation2d.fromDegrees(180))
          .transformBy(BARGE_OFFSET);
}
