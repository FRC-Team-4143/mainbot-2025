package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.ElevatorTargets.TargetType;
import frc.lib.FieldRegions;
import frc.lib.ScoringPoses;
import frc.mw_lib.subsystem.Subsystem;
import frc.mw_lib.util.Util;
import frc.robot.Constants;
import frc.robot.subsystems.Claw.ClawMode;
import java.util.Optional;
import monologue.Annotations.Log;
import monologue.Logged;

public class GameStateManager extends Subsystem {

  private StructPublisher<Pose2d> reef_target_pub_;
  private StructArrayPublisher<Pose3d> target_stages_pub_;
  private StructPublisher<Pose3d> target_arm_pub_;

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
    END,
    TELEOP_CONTROL
  }

  public enum ReefScoringTarget {
    L1,
    L2,
    L2_FAR,
    L3,
    L3_FAR,
    L4,
    ALGAE,
    TURTLE,
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

    // Mechanism Setup
    reef_target_pub_ =
        NetworkTableInstance.getDefault()
            .getStructTopic("GameStateManager/Reef Target", Pose2d.struct)
            .publish();
    target_stages_pub_ =
        NetworkTableInstance.getDefault()
            .getStructArrayTopic("GameStateManager/Elevator Stages", Pose3d.struct)
            .publish();
    target_arm_pub_ =
        NetworkTableInstance.getDefault()
            .getStructTopic("GameStateManager/Arm", Pose3d.struct)
            .publish();
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
    io_.reef_target_ = reefPose(io_.target_column_);
    switch (io_.robot_state_) {
      case TARGET_ACQUISITION:
        if (io_.reef_target_.isPresent()) {
          SwerveDrivetrain.getInstance().setTargetPose(io_.reef_target_.get());
          if (FieldRegions.REEF_ENTER.contains(PoseEstimator.getInstance().getRobotPose())) {
            io_.robot_state_ = RobotState.APPROACHING_TARGET;
          }
        }
        break;
      case APPROACHING_TARGET:
        updateNeededCoralOffset();
        SwerveDrivetrain.getInstance().setTargetPose(io_.reef_target_.get());
        if (Util.epislonEquals(
            PoseEstimator.getInstance().getRobotPose().getRotation(),
            io_.reef_target_.get().getRotation(),
            Constants.GameStateManagerConstants.REQUIRED_ROTATION_FOR_ELEVATOR)) {
          // move elevator once within rotation threshold
          Elevator.getInstance().setTarget(elevatorTargetSwitch());
        }
        if (SwerveDrivetrain.getInstance().atTractorBeamPose()
            && Elevator.getInstance().isElevatorAndArmAtTarget()) {
          // Once at final target, hand off control
          SwerveDrivetrain.getInstance().restoreDefaultDriveMode();
          io_.robot_state_ = RobotState.SCORING;
        }
        break;
      case SCORING:
        if (Claw.getInstance().isCoralMode()) {
          Claw.getInstance().setClawMode(ClawMode.SHOOT);
        }
        if (!FieldRegions.REEF_EXIT.contains(PoseEstimator.getInstance().getRobotPose())) {
          // wait until you leave the exit Circle
          io_.robot_state_ = RobotState.END;
        }
        break;
      case END:
        // clear saved vars and reset drive mode
        SwerveDrivetrain.getInstance().restoreDefaultDriveMode();
        Claw.getInstance().setClawMode(ClawMode.IDLE);
        io_.robot_state_ = RobotState.TELEOP_CONTROL;
        Claw.getInstance().disableBlastMode();
        break;
      case TELEOP_CONTROL:
        // normal control
      default:
        break;
    }
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
    SmartDashboard.putBoolean("Subsystems/GameStateManager/isRunning", isRunning());
    SmartDashboard.putString(
        "Subsystems/GameStateManager/Robot State", io_.robot_state_.toString());
    SmartDashboard.putString(
        "Subsystems/GameStateManager/Target Colum", io_.target_column_.toString());
    SmartDashboard.putString(
        "Subsystems/GameStateManager/Saved Colum", io_.saved_target_column_.toString());
    SmartDashboard.putString(
        "Subsystems/GameStateManager/Target Level", io_.scoring_target_.toString());
    SmartDashboard.putString(
        "Subsystems/GameStateManager/Saved Target Level", io_.saved_scoring_target_.toString());
    SmartDashboard.putBoolean("Subsystems/GameStateManager/Ready to Score", isReadyToScore());

    if (io_.reef_target_.isPresent()) {
      reef_target_pub_.set(io_.reef_target_.get());
      Elevator.getInstance()
          .updateMechanism(
              target_stages_pub_,
              target_arm_pub_,
              Elevator.getInstance()
                  .getElevatorKinematics()
                  .translationToJointSpace(elevatorTargetSwitch().getTarget().getTranslation()));
    }
  }

  public boolean isReadyToScore() {
    return SwerveDrivetrain.getInstance().atTractorBeamPose()
        && Elevator.getInstance().isElevatorAndArmAtTarget()
        && io_.robot_state_ == RobotState.SCORING;
  }

  /**
   * @param target_score new target
   * @param save weather or not to save the target
   */
  public void setScoringColum(Column col, boolean save) {
    io_.target_column_ = col;
    if (save) {
      io_.saved_target_column_ = col;
    }
  }

  /**
   * @param target_score new target
   * @param save weather or not to save the target
   */
  public void setScoringTarget(ReefScoringTarget target, boolean save) {
    io_.scoring_target_ = target;
    if (save) {
      io_.saved_scoring_target_ = target;
    }
  }

  public void setScoringObj(Column col, ReefScoringTarget target, boolean save) {
    setScoringColum(col, save);
    setScoringTarget(target, save);
  }

  public static Command setScoringCommand(Column col, ReefScoringTarget target) {
    return Commands.runOnce(() -> getInstance().setScoringObj(col, target, true));
  }

  public ReefScoringTarget getSavedScoringTarget() {
    return io_.saved_scoring_target_;
  }

  public Column getSavedScoringColum() {
    return io_.saved_target_column_;
  }

  /**
   * Returns a target pose when robot is in an reef region If not in a region empty is returned.
   * Also adjust to the provided column
   *
   * @return target pose
   */
  public Optional<Pose2d> reefPose(Column column) {
    for (int i = 0; i < FieldRegions.REEF_REGIONS.length; i++) {
      if (FieldRegions.REEF_REGIONS[i].contains(PoseEstimator.getInstance().getRobotPose())) {
        if (io_.scoring_target_ == ReefScoringTarget.L1) {
          return Optional.of(
              FieldRegions.REGION_POSE_TABLE
                  .get(FieldRegions.REEF_REGIONS[i].getName())
                  .transformBy(ScoringPoses.ALGAE_ALIGN_OFFSET));
          // TODO: Make Coral Align Offset
        }
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
          io_.algae_level_high = ((i % 2) == 0); // this line prevents converting this method to use
          return Optional.of(
              FieldRegions.REGION_POSE_TABLE
                  .get(FieldRegions.REEF_REGIONS[i].getName())
                  .transformBy(ScoringPoses.ALGAE_ALIGN_OFFSET));
        }
      }
    }
    return Optional.empty();
  }

  public void setRobotState(RobotState state) {
    // sets the current state of the robot (should really only set to
    // TARGET_ACQUISITION or END to preserve
    // structure of the switch case)
    io_.robot_state_ = state;
  }

  public RobotState getRobotState() {
    return io_.robot_state_;
  }

  public TargetType elevatorTargetSwitch() {
    switch (io_.scoring_target_) {
      case L1:
        return TargetType.L1;
      case L2:
        return TargetType.L2;
      case L2_FAR:
        return TargetType.L2_FAR;
      case L3:
        return TargetType.L3;
      case L3_FAR:
        return TargetType.L3_FAR;
      case L4:
        return TargetType.L4;
      case ALGAE:
        if (io_.algae_level_high) {
          return TargetType.ALGAE_HIGH;
        } else {
          return TargetType.ALGAE_LOW;
        }
      case TELEOP_CONTROL:
      case TURTLE:
      default:
        return (Claw.getInstance().isCoralMode()) ? TargetType.CORAL_STOW : TargetType.ALGAE_STOW;
    }
  }

  public void updateNeededCoralOffset() {
    if (io_.reef_target_.isPresent()
        && SwerveDrivetrain.getInstance().getFailingToReachTarget()
        && SwerveDrivetrain.getInstance().getTractorBeamError()
            <= Constants.GameStateManagerConstants.CORAL_BLOCKED_THRESHOLD) {
      if (io_.scoring_target_ == ReefScoringTarget.L2) {
        io_.reef_target_ =
            Optional.of(io_.reef_target_.get().transformBy(ScoringPoses.CORAL_OFFSET));
        io_.scoring_target_ = ReefScoringTarget.L2_FAR;
      } else if (io_.scoring_target_ == ReefScoringTarget.L3) {
        io_.reef_target_ =
            Optional.of(io_.reef_target_.get().transformBy(ScoringPoses.CORAL_OFFSET));
        io_.scoring_target_ = ReefScoringTarget.L3_FAR;
      }
    }
  }

  public boolean isRunning() {
    return !(io_.robot_state_ == RobotState.TELEOP_CONTROL || io_.robot_state_ == RobotState.END);
  }

  /**
   * Inside this function actuator OUTPUTS should be updated from data contained in the PeriodicIO
   * class defined below. There should be little to no logic contained within this function, and no
   * sensors should be read.
   */
  public class GameStateManagerPeriodicIo implements Logged {
    @Log.File public ReefScoringTarget scoring_target_ = ReefScoringTarget.TURTLE;
    @Log.File public ReefScoringTarget saved_scoring_target_ = ReefScoringTarget.L2;
    @Log.File public RobotState robot_state_ = RobotState.TELEOP_CONTROL;
    @Log.File public Optional<Pose2d> reef_target_ = Optional.empty();
    @Log.File public Column target_column_ = Column.LEFT;
    @Log.File public Column saved_target_column_ = Column.LEFT;

    @Log.File
    public boolean algae_level_high = false; // false is low level and true is the higher level
  }

  @Override
  public Logged getLoggingObject() {
    return io_;
  }
}
