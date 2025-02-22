package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.FieldRegions;
import frc.lib.ReefSectionState;
import frc.lib.ScoringPoses;
import frc.mw_lib.geometry.PolygonRegion;
import frc.mw_lib.subsystem.Subsystem;
import frc.mw_lib.util.Util;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator.SpeedLimit;
import frc.robot.subsystems.SwerveDrivetrain.DriveMode;
import java.util.ArrayList;
import java.util.Optional;
import monologue.Annotations.Log;
import monologue.Logged;

public class GameStateManager extends Subsystem {

  private StructPublisher<Pose2d> reef_target_publisher =
      NetworkTableInstance.getDefault().getStructTopic("ReefTarget", Pose2d.struct).publish();

  Elevator elevator_;
  PoseEstimator poseEstimator_;
  SwerveDrivetrain drivetrain_;

  // Singleton pattern
  private static GameStateManager instance_ = null;

  // C
  public static GameStateManager getInstance() {
    if (instance_ == null) {
      instance_ = new GameStateManager();
    }
    return instance_;
  }

  /** Class Members */
  private GameStateManagerPeriodicIo io_;

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
    io_ = new GameStateManagerPeriodicIo();
    elevator_ = Elevator.getInstance();
    poseEstimator_ = PoseEstimator.getInstance();
    drivetrain_ = SwerveDrivetrain.getInstance();
  }

  /**
   * This function should be logic and code to fully reset your subsystem. This is called during
   * initialization, and should handle I/O configuration and initializing data members.
   */
  @Override
  public void reset() {}

  /**
   * Inside this function, all of the SENSORS should be read into variables stored in the PeriodicIO
   * class defined below. There should be no calls to output to actuators, or any logic within this
   * function.
   */
  @Override
  public void readPeriodicInputs(double timestamp) {}

  /**
   * Inside this function, all of the LOGIC should compute updates to output variables in the
   * PeriodicIO class defined below. There should be no calls to read from sensors or write to
   * actuators in this function.
   */
  @Override
  public void updateLogic(double timestamp) {

    io_.intent = updateIntent();
    if (io_.reef_target.isPresent()) {
      reef_target_publisher.set(io_.reef_target.get());
    }

    switch (io_.robot_state_) {
      case TARGET_ACQUISITION:
        io_.reef_target = reefPose(Column.CENTER);
        if (io_.reef_target.isPresent()
            && (io_.intent == Intent.INTAKE_ALGAE || io_.intent == Intent.SCORE_CORAL)) {
          drivetrain_.setTargetPose(io_.reef_target.get());
          if (FieldRegions.REEF_ENTER.contains(poseEstimator_.getRobotPose())) {
            io_.robot_state_ = RobotState.APPROACHING_TARGET;
          }
        }
        break;
      case APPROACHING_TARGET:
        elevatorTargetSwitch();
        // need to fix that switch cases were methods
        if (io_.use_cam_for_reef_state && io_.target_column.isEmpty()) {
          if (io_.reef_section_list.size() < io_.num_frames) {
            // add reef states to list while driving to the center
            io_.reef_section_list.add(getNextReefFrame());
            io_.reef_target = reefPose(Column.CENTER);
            drivetrain_.setTargetPose(io_.reef_target.get());
          } else {
            // avg states and set final target
            io_.reef_section_state =
                Optional.of(ReefSectionState.averageReefSections(io_.reef_section_list, 0.50));
            io_.target_column = findReefTargetColumn();
          }
        }
        if (io_.target_column.isPresent()) {
          // drive towards final target
          io_.reef_target = reefPose(io_.target_column.get());
          drivetrain_.setTargetPose(io_.reef_target.get());
          if (Util.epislonEquals(
              poseEstimator_.getRobotPose(), io_.reef_target.get(), 0.0873, 0.0508)) {
            // Once at final target, hand off control
            // drivetrain_.setDriveMode(DriveMode.FIELD_CENTRIC);
            io_.robot_state_ = RobotState.SCORING;
            // CommandScheduler.getInstance().schedule(new Score().withTimeout(1));
          }
        }
        break;
      case SCORING:
        drivetrain_.setDriveMode(DriveMode.FIELD_CENTRIC);
        // wait until you leave the exit Circle
        if (!FieldRegions.REEF_EXIT.contains(poseEstimator_.getRobotPose())) {
          io_.robot_state_ = RobotState.END;
        }
        break;
      case END:
        // clear saved vars and reset drive mode
        drivetrain_.setDriveMode(DriveMode.FIELD_CENTRIC);
        // target_column = Optional.empty();
        io_.reef_section_state = Optional.empty();
        io_.robot_state_ = RobotState.TELEOP_CONTROL;
        if (Claw.getInstance().isCoralMode()) {
          elevator_.setSpeedLimit(SpeedLimit.CORAL);
          elevator_.setTarget(ElevatorConstants.Target.STOW);
        } else {
          elevator_.setSpeedLimit(SpeedLimit.ALGAE);
          elevator_.setTarget(ElevatorConstants.Target.ALGAE_STOW);
        }
        break;
      case TELEOP_CONTROL:
        // normal control
      default:
        break;
    }

    // robotStateSwitch(); need to fix that switch cases were methods

  }

  /**
   * Inside this function actuator OUTPUTS should be updated from data contained in the PeriodicIO
   * class defined below. There should be little to no logic contained within this function, and no
   * sensors should be read.
   */
  @Override
  public void writePeriodicOutputs(double timestamp) {}

  /**
   * Inside this function telemetry should be output to smartdashboard. The data should be collected
   * out of the PeriodicIO class instance defined below. There should be no sensor information read
   * in this function nor any outputs made to actuators within this function. Only publish to
   * smartdashboard here.
   */
  @Override
  public void outputTelemetry(double timestamp) {
    SmartDashboard.putString("Robot State", io_.robot_state_.toString());
    SmartDashboard.putString("Target Colum", io_.target_column.toString());
    SmartDashboard.putString("Intent", io_.intent.toString());
    SmartDashboard.putString("Target Level", io_.scoring_target.toString());
  }

  public void wantedTarget(ScoringTarget target_score) {
    io_.scoring_target = target_score;
  }

  /**
   * Returns a target pose when robot is in an load station region If not in a region empty is
   * returned.
   *
   * @return target pose
   */
  public Optional<Pose2d> loadStationPose() {
    for (PolygonRegion region : FieldRegions.STATION_REGIONS) {
      if (region.contains(poseEstimator_.getRobotPose())) {
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
    for (int i = 0; i < FieldRegions.REEF_REGIONS.length; i++) {
      if (FieldRegions.REEF_REGIONS[i].contains(poseEstimator_.getRobotPose())) {
        if (column == Column.CENTER) {
          return Optional.of(
              FieldRegions.REGION_POSE_TABLE.get(FieldRegions.REEF_REGIONS[i].getName()));
        }
        if (column == Column.LEFT) {
          return Optional.of(
              FieldRegions.REGION_POSE_TABLE
                  .get(FieldRegions.REEF_REGIONS[i].getName())
                  .transformBy(ScoringPoses.LEFT_COLUMN_OFFSET));
        }

        if (column == Column.RIGHT) {
          return Optional.of(
              FieldRegions.REGION_POSE_TABLE
                  .get(FieldRegions.REEF_REGIONS[i].getName())
                  .transformBy(ScoringPoses.RIGHT_COLUMN_OFFSET));
        }
        if (column == Column.ALGAE) {
          io_.algae_level_high = ((i % 2) == 0);
          return Optional.of(
              FieldRegions.REGION_POSE_TABLE
                  .get(FieldRegions.REEF_REGIONS[i].getName())
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
      if (region.contains(poseEstimator_.getRobotPose())) {
        return Optional.of(FieldRegions.REGION_POSE_TABLE.get(region.getName()));
      }
    }
    return Optional.empty();
  }

  public void setSelectedReefLevel(int level) {
    // sets the scoring level of the reef
    io_.selected_reef_level = level;
  }

  public void setRobotState(RobotState state) {
    // sets the current state of the robot (should really only set to
    // TARGET_ACQUISITION to preserve
    // structure of the switch case)
    io_.robot_state_ = state;
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
    // sets the target column, will be overwritten if use_cam_for_reef_state is true
    io_.target_column = Optional.of(column);
  }

  public Optional<Column> findReefTargetColumn() {
    // use reef_section_state && selected_reef_level to find open slot and return
    // what column it is in
    // or none if there is no open slot at the specified level
    return Optional.of(Column.LEFT);
  }

  public void elevatorTargetSwitch() {
    switch (io_.scoring_target) {
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
        if (io_.algae_level_high) {
          elevator_.setTarget(ElevatorConstants.Target.ALGAE_HIGH);
        } else {
          elevator_.setTarget(ElevatorConstants.Target.ALGAE_LOW);
        }
        break;
      case TURTLE:
      default:
        elevator_.setTarget(ElevatorConstants.Target.STOW);
        break;
      case TELEOP_CONTROL:
        break;
    }
  }

  /**
   * Inside this function actuator OUTPUTS should be updated from data contained in the PeriodicIO
   * class defined below. There should be little to no logic contained within this function, and no
   * sensors should be read.
   */
  public class GameStateManagerPeriodicIo implements Logged {

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

    @Log.File
    private boolean algae_level_high = false; // false is low level and true is the higher level
  }

  @Override
  public Logged getLoggingObject() {
    return io_;
  }
}
