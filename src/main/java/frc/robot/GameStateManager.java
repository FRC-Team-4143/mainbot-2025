package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.FieldRegions;
import frc.lib.ReefSectionState;
import frc.mw_lib.geometry.PolygonRegion;
import frc.mw_lib.util.Util;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.Score;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.SwerveDrivetrain.DriveMode;
import java.util.ArrayList;
import java.util.Optional;
import monologue.Annotations.Log;

public class GameStateManager {

  // Singleton pattern
  private static GameStateManager instance_ = null;

  public static GameStateManager getInstance() {
    if (instance_ == null) {
      instance_ = new GameStateManager();
    }
    return instance_;
  }

  @Log.File private static ScoringTarget scoring_target = ScoringTarget.TURTLE;
  @Log.File private static ReefSectionState reef_section_state = new ReefSectionState();

  @Log.File
  private static ArrayList<ReefSectionState> reef_section_list = new ArrayList<ReefSectionState>();

  @Log.File private static RobotState robot_state_ = RobotState.TELEOP_CONTROL;
  @Log.File private static Optional<Pose2d> reef_target = Optional.empty();
  @Log.File private static Optional<Pose2d> station_target = Optional.empty();
  @Log.File private static Optional<Pose2d> algae_target = Optional.empty();
  @Log.File private static int selected_reef_level = 2;
  @Log.File private static Intent intent = Intent.SCORE_CORAL;
  @Log.File private static int num_frames = 5;
  @Log.File private static boolean reef_state_found = false;
  @Log.File private static Optional<Column> target_column = Optional.empty();

  private StructPublisher<Pose2d> reef_target_publisher =
      NetworkTableInstance.getDefault().getStructTopic("ReefTarget", Pose2d.struct).publish();

  static Elevator elevator_;
  static PoseEstimator poseEstimator_;
  static SwerveDrivetrain drivetrain_;

  public static enum RobotState {
    TARGET_ACQUISITION,
    APPROACHING_TARGET,
    SCORING,
    LEAVING,
    TELEOP_CONTROL,
    END
  }

  public static enum Intent {
    INTAKE_CORAL,
    INTAKE_ALGAE,
    SCORE_CORAL,
    SCORE_ALGAE
  }

  public static enum ScoringTarget {
    TURTLE,
    REEF_L1,
    REEF_L2,
    REEF_L3,
    REEF_L4,
    REEF_ALGAE,
    BARGE,
    PROCESSOR,
    SOURCE,
    TELEOP_CONTROL
  }

  public static enum Column {
    LEFT,
    RIGHT,
    CENTER
  }

  private GameStateManager() {
    elevator_ = Elevator.getInstance();
    poseEstimator_ = PoseEstimator.getInstance();
    drivetrain_ = SwerveDrivetrain.getInstance();
  }

  public void updateGameState() {
    intent = updateIntent();
    if (reef_target.isPresent()) {
      reef_target_publisher.set(reef_target.get());
    }
    robotStateSwitch();
  }

  public static void robotStateSwitch() {
    switch (robot_state_) {
      case TARGET_ACQUISITION:
        reef_target = reefPose(Column.CENTER);
        if (reef_target.isPresent()
            && (intent == Intent.INTAKE_ALGAE || intent == Intent.SCORE_CORAL)) {
          drivetrain_.setTargetPose(reef_target.get());
          if (FieldRegions.REEF_ENTER.contains(poseEstimator_.getFieldPose())) {
            reef_state_found = false;
            target_column = Optional.empty();
            robot_state_ = RobotState.APPROACHING_TARGET;
          }
        }
        break;
      case APPROACHING_TARGET:
        // move elevator to level
        if (reef_section_list.size() < num_frames) {
          reef_section_list.add(getNextReefFrame());
          reef_section_state = ReefSectionState.averageReefSections(reef_section_list, 0.50);
          reef_target = reefPose(Column.CENTER);
          drivetrain_.setTargetPose(reef_target.get());
        } else {
          reef_state_found = true;
          target_column = findReefTargetColumn();
        }
        if (target_column.isPresent()) {
          reef_target = reefPose(target_column.get());
          drivetrain_.setTargetPose(reef_target.get());
          if (Util.epislonEquals(
              poseEstimator_.getFieldPose(), reef_target.get(), 0.0873, 0.0508)) {
            drivetrain_.setDriveMode(DriveMode.FIELD_CENTRIC);
            robot_state_ = RobotState.SCORING;
          }
        }
        break;
      case SCORING:
        if (!FieldRegions.REEF_EXIT.contains(poseEstimator_.getFieldPose())) {
          // set elevator to idle state
          //CommandScheduler.getInstance().schedule(new Score().withTimeout(1));
          robot_state_ = RobotState.END;
        }
        break;
      case END:
        drivetrain_.setDriveMode(DriveMode.FIELD_CENTRIC);
        scoring_target = ScoringTarget.TELEOP_CONTROL;
        break;
      case TELEOP_CONTROL:
      default:
        break;
    }
  }

  public static void elevatorTargetSwitch() {
    switch (scoring_target) {
      case REEF_L2:
        elevator_.setTarget(ElevatorConstants.Target.L2);
        break;
      case REEF_L3:
        elevator_.setTarget(ElevatorConstants.Target.L3);
        break;
      case REEF_L4:
        elevator_.setTarget(ElevatorConstants.Target.L4);
        break;
      case REEF_ALGAE:
        // ask where algae is then go to that height
        break;
      case TURTLE:
      default:
        elevator_.setTarget(ElevatorConstants.Target.STOW);
        break;
      case TELEOP_CONTROL:
        break;
    }
  }

  public static void wantedTarget(ScoringTarget target_score) {
    scoring_target = target_score;
  }

  /**
   * Returns a target pose when robot is in an load station region If not in a region empty is
   * returned.
   *
   * @return target pose
   */
  public static Optional<Pose2d> loadStationPose() {
    for (PolygonRegion region : FieldRegions.STATION_REGIONS) {
      if (region.contains(poseEstimator_.getFieldPose())) {
        return Optional.of(FieldRegions.REGION_POSE_TABLE.get(region.getName()));
      }
    }
    return Optional.empty();
  }

  /**
   * Returns a target pose when robot is in an reef region If not in a region empty is returned.
   *
   * @return target pose
   */
  public static Optional<Pose2d> reefPose(Column column) {
    for (PolygonRegion region : FieldRegions.REEF_REGIONS) {
      if (region.contains(poseEstimator_.getFieldPose())) {
        if (column == Column.CENTER) {
          return Optional.of(FieldRegions.REGION_POSE_TABLE.get(region.getName()));
        }
        if (column == Column.LEFT) {
          return Optional.of(
              FieldRegions.REGION_POSE_TABLE
                  .get(region.getName())
                  .transformBy(new Transform2d(0, 0.165, new Rotation2d())));
        }
        if (column == Column.RIGHT) {
          return Optional.of(
              FieldRegions.REGION_POSE_TABLE
                  .get(region.getName())
                  .transformBy(new Transform2d(0, -0.165, new Rotation2d())));
        }
      }
    }
    return Optional.empty();
  }

  /**
   * Returns a target pose when robot is in an algae region If not in a region empty is returned.
   *
   * @return target pose
   */
  public static Optional<Pose2d> algaePose() {
    for (PolygonRegion region : FieldRegions.ALGAE_REGIONS) {
      if (region.contains(poseEstimator_.getFieldPose())) {
        return Optional.of(FieldRegions.REGION_POSE_TABLE.get(region.getName()));
      }
    }
    return Optional.empty();
  }

  public static void setSelectedReefLevel(int level) {
    selected_reef_level = level;
  }

  public static void setRobotState(RobotState state) {
    robot_state_ = state;
  }

  public static Intent updateIntent() {
    return Intent.SCORE_CORAL;
  }

  public static ReefSectionState getNextReefFrame() {
    return new ReefSectionState();
  }

  public static Optional<Column> findReefTargetColumn() {
    // use reef_section_state && selected_reef_level to find open slot
    return Optional.of(Column.LEFT);
  }
}
