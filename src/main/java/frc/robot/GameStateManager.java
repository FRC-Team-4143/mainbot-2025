package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import frc.lib.FieldRegions;
import frc.lib.ReefSectionState;
import frc.mw_lib.geometry.PolygonRegion;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.SwerveDrivetrain;
import java.util.Optional;
import monologue.Annotations.Log;

public class GameStateManager {

  @Log.File private static ScoringTarget scoring_target_ = ScoringTarget.TURTLE;
  @Log.File private static ReefSectionState reef_section_state_ = new ReefSectionState();
  @Log.File private static RobotState robot_state_ = RobotState.TELEOP_CONTROL;
  @Log.File private static Optional<Pose2d> reef_target = Optional.empty();
  @Log.File private static Optional<Pose2d> station_target = Optional.empty();
  @Log.File private static Optional<Pose2d> algae_target = Optional.empty();

  static Elevator elevator_;
  static PoseEstimator poseEstimator_;
  static SwerveDrivetrain drivetrain_;

  public static enum RobotState {
    TARGET_ACQUISITION,
    APPROACHING_TARGET,
    SCORING,
    LEAVING,
    TELEOP_CONTROL
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
    IDLE
  }

  public GameStateManager() {}

  public static void updateGameState() {
    reef_target = reefPose();
    station_target = loadStationPose();
    algae_target = algaePose();
    robotStateSwitch();
  }

  public static void robotStateSwitch() {
    switch (robot_state_) {
      case TARGET_ACQUISITION:
        if (reef_target.isPresent()) {
          drivetrain_.setTargetPose(reef_target.get());
        }
        break;
      case APPROACHING_TARGET:
        // elevatorTargetSwitch();
        // look for pieces and go to location that does not have a piece
        // call get REEF_ALGAE to manage where the elevator goes if going after algae
        break;
      case SCORING:
        // turns off tractor beam
        break;
      case LEAVING:
        // set elevator to idle state
        break;
      case TELEOP_CONTROL:
      default:
        scoring_target_ = ScoringTarget.IDLE;
        break;
    }
  }

  public static void elevatorTargetSwitch() {
    switch (scoring_target_) {
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
      case IDLE:
        break;
    }
  }

  public static void wantedTarget(ScoringTarget target_score) {
    scoring_target_ = target_score;
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
  public static Optional<Pose2d> reefPose() {
    for (PolygonRegion region : FieldRegions.REEF_REGIONS) {
      if (region.contains(poseEstimator_.getFieldPose())) {
        return Optional.of(FieldRegions.REGION_POSE_TABLE.get(region.getName()));
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
}
