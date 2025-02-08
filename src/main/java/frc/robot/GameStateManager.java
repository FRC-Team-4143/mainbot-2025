package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import frc.lib.FieldRegions;
import frc.lib.ReefSectionState;
import frc.mw_lib.geometry.PolygonRegion;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.SwerveDrivetrain;
import java.util.Optional;

public class GameStateManager {

  private static ScoringTarget scoring_target_ = ScoringTarget.TURTLE;
  private static ReefSectionState reef_section_state_ = new ReefSectionState();
  private static RobotState robot_state_ = RobotState.TELEOP_CONTROL;
  private static Optional<Pose2d> reef_target;
  private static Optional<Pose2d> station_target;
  private static Optional<Pose2d> algae_target;

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

  public static enum GamePieceConfig {}

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
          drivetrain_.tractorBeam(reef_target.get());
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
        elevator_.setTarget(Constants.ElevatorConstants.Target.L2);
        break;
      case REEF_L3:
        elevator_.setTarget(Constants.ElevatorConstants.Target.L3);
        break;
      case REEF_L4:
        elevator_.setTarget(Constants.ElevatorConstants.Target.L4);
        break;
      case REEF_ALGAE:
        // ask where algae is then go to that height
        break;
      case TURTLE:
      default:
        elevator_.setTarget(Constants.ElevatorConstants.Target.STOW);
        break;
      case IDLE:
        break;
    }
  }

  // public static Optional<Pose2d> findTarget() {

  // }

  public static void wantedTarget(ScoringTarget target_score) {
    scoring_target_ = target_score;
  }

  public static Optional<Pose2d> loadStationPose() {
    for (PolygonRegion loadStations : FieldRegions.STATION_REGIONS) {
      if (loadStations.contains(poseEstimator_.getFieldPose())) {
        // && do not have any coral or algae

      }
    }
    return Optional.empty();
  }

  public static Optional<Pose2d> reefPose() {
    for (PolygonRegion reefFaces : FieldRegions.REEF_REGIONS) {
      if (reefFaces.contains(poseEstimator_.getFieldPose())) {}
    }
    return Optional.empty();
  }

  public static Optional<Pose2d> algaePose() {
    for (PolygonRegion algae : FieldRegions.ALGAE_REGIONS) {
      if (algae.contains(poseEstimator_.getFieldPose())) {
        // && have an algae

      }
    }
    return Optional.empty();
  }
}
