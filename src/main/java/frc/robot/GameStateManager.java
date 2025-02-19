package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.FieldRegions;
import frc.lib.ReefSectionState;
import frc.lib.ScoringPoses;
import frc.mw_lib.geometry.PolygonRegion;
import frc.mw_lib.util.Util;
import frc.robot.Constants.ElevatorConstants;
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

  @Log.File private ScoringTarget scoring_target = ScoringTarget.TURTLE;
  @Log.File private Optional<ReefSectionState> reef_section_state = Optional.empty();

  @Log.File
  private ArrayList<ReefSectionState> reef_section_list = new ArrayList<ReefSectionState>();

  @Log.File private RobotState robot_state_ = RobotState.TELEOP_CONTROL;
  @Log.File private Optional<Pose2d> reef_target = Optional.empty();
  @Log.File private Optional<Pose2d> station_target = Optional.empty();
  @Log.File private Optional<Pose2d> algae_target = Optional.empty();
  @Log.File private int selected_reef_level = 2;
  @Log.File private Intent intent = Intent.SCORE_CORAL;
  @Log.File private int num_frames = 5;
  @Log.File private boolean use_cam_for_reef_state = false;
  @Log.File private Optional<Column> target_column = Optional.empty();

  private StructPublisher<Pose2d> reef_target_publisher =
      NetworkTableInstance.getDefault().getStructTopic("ReefTarget", Pose2d.struct).publish();

  Elevator elevator_;
  PoseEstimator poseEstimator_;
  SwerveDrivetrain drivetrain_;

  public enum RobotState {
    TARGET_ACQUISITION,
    APPROACHING_TARGET,
    SCORING,
    LEAVING,
    TELEOP_CONTROL,
    END
  }

  public enum Intent {
    INTAKE_CORAL,
    INTAKE_ALGAE,
    SCORE_CORAL,
    SCORE_ALGAE
  }

  public enum ScoringTarget {
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

  public enum Column {
    LEFT,
    RIGHT,
    CENTER,
    ALGAE
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
    SmartDashboard.putString("Robot State", robot_state_.toString());
    SmartDashboard.putString("Target Colum", target_column.toString());
    SmartDashboard.putString("Intent", intent.toString());
    SmartDashboard.putString("Target Level", scoring_target.toString());
  }

  public void robotStateSwitch() {
    switch (robot_state_) {
      case TARGET_ACQUISITION:
        reef_target = reefPose(Column.CENTER);
        if (reef_target.isPresent()
            && (intent == Intent.INTAKE_ALGAE || intent == Intent.SCORE_CORAL)) {
          drivetrain_.setTargetPose(reef_target.get());
          if (FieldRegions.REEF_ENTER.contains(poseEstimator_.getFieldPose())) {
            robot_state_ = RobotState.APPROACHING_TARGET;
          }
        }
        break;
      case APPROACHING_TARGET:
        elevatorTargetSwitch();
        if (use_cam_for_reef_state && target_column.isEmpty()) {
          if (reef_section_list.size() < num_frames) {
            // add reef states to list while driveing to the center
            reef_section_list.add(getNextReefFrame());
            reef_target = reefPose(Column.CENTER);
            drivetrain_.setTargetPose(reef_target.get());
          } else {
            // avg states and set final target
            reef_section_state =
                Optional.of(ReefSectionState.averageReefSections(reef_section_list, 0.50));
            target_column = findReefTargetColumn();
          }
        }
        if (target_column.isPresent()) {
          // drive twards final target
          reef_target = reefPose(target_column.get());
          drivetrain_.setTargetPose(reef_target.get());
          if (Util.epislonEquals(
              poseEstimator_.getFieldPose(), reef_target.get(), 0.0873, 0.0508)) {
            // Once at final target, hand off contol
            // drivetrain_.setDriveMode(DriveMode.FIELD_CENTRIC);
            robot_state_ = RobotState.SCORING;
            // CommandScheduler.getInstance().schedule(new Score().withTimeout(1));
          }
        }
        break;
      case SCORING:
        // wait until you leave the exit Circle
        if (!FieldRegions.REEF_EXIT.contains(poseEstimator_.getFieldPose())) {
          elevator_.setTarget(ElevatorConstants.Target.STOW);
          robot_state_ = RobotState.END;
        }
        break;
      case END:
        // clear saved vars and reset drive mode
        drivetrain_.setDriveMode(DriveMode.FIELD_CENTRIC);
        // target_column = Optional.empty();
        reef_section_state = Optional.empty();
        robot_state_ = RobotState.TELEOP_CONTROL;
        break;
      case TELEOP_CONTROL:
        // normal control
      default:
        break;
    }
  }

  public void elevatorTargetSwitch() {
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

  public void wantedTarget(ScoringTarget target_score) {
    scoring_target = target_score;
  }

  /**
   * Returns a target pose when robot is in an load station region If not in a region empty is
   * returned.
   *
   * @return target pose
   */
  public Optional<Pose2d> loadStationPose() {
    for (PolygonRegion region : FieldRegions.STATION_REGIONS) {
      if (region.contains(poseEstimator_.getFieldPose())) {
        return Optional.of(FieldRegions.REGION_POSE_TABLE.get(region.getName()));
      }
    }
    return Optional.empty();
  }

  /**
   * Returns a target pose when robot is in an reef region If not in a region empty is returned.
   * Also adjust to the provided column
   *
   * @return target pose
   */
  public Optional<Pose2d> reefPose(Column column) {
    for (PolygonRegion region : FieldRegions.REEF_REGIONS) {
      if (region.contains(poseEstimator_.getFieldPose())) {
        if (column == Column.CENTER) {
          return Optional.of(FieldRegions.REGION_POSE_TABLE.get(region.getName()));
        }
        if (column == Column.LEFT) {
          return Optional.of(
              FieldRegions.REGION_POSE_TABLE
                  .get(region.getName())
                  .transformBy(ScoringPoses.LEFT_COLUMN_OFFEST));
        }
        if (column == Column.RIGHT) {
          return Optional.of(
              FieldRegions.REGION_POSE_TABLE
                  .get(region.getName())
                  .transformBy(ScoringPoses.RIGHT_COLUMN_OFFSET));
        }
        if (column == Column.ALGAE) {
          return Optional.of(
              FieldRegions.REGION_POSE_TABLE
                  .get(region.getName())
                  .transformBy(ScoringPoses.ALGAE_ALIGN_OFFSET));
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
  public Optional<Pose2d> algaePose() {
    for (PolygonRegion region : FieldRegions.ALGAE_REGIONS) {
      if (region.contains(poseEstimator_.getFieldPose())) {
        return Optional.of(FieldRegions.REGION_POSE_TABLE.get(region.getName()));
      }
    }
    return Optional.empty();
  }

  public void setSelectedReefLevel(int level) {
    // sets the scoring level of the reef
    selected_reef_level = level;
  }

  public void setRobotState(RobotState state) {
    // sets the current state of the robot (should really only set to TARGET_ACQUISITION to preserve
    // structure of the switch case)
    robot_state_ = state;
  }

  public Intent updateIntent() {
    // use current game pice mode and possession to find intent
    return Intent.SCORE_CORAL;
  }

  public ReefSectionState getNextReefFrame() {
    // ask the cam for its next solve of the reef state
    return new ReefSectionState();
  }

  public void setTargetColumn(Column column) {
    // sets the target column, will be overwriten if use_cam_for_reef_state is true
    target_column = Optional.of(column);
  }

  public Optional<Column> findReefTargetColumn() {
    // use reef_section_state && selected_reef_level to find open slot and return
    // what column it is in
    // or none if there is no open slot at the specified level
    return Optional.of(Column.LEFT);
  }
}
