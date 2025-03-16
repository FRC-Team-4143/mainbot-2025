package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.ElevatorTargets.Target;
import frc.lib.FieldRegions;
import frc.lib.ScoringPoses;
import frc.mw_lib.subsystem.Subsystem;
import frc.mw_lib.util.Util;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator.SpeedLimit;
import java.util.Optional;
import monologue.Annotations.Log;
import monologue.Logged;

public class GameStateManager extends Subsystem {

  private StructPublisher<Pose2d> reef_target_publisher =
      NetworkTableInstance.getDefault().getStructTopic("ReefTarget", Pose2d.struct).publish();

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
    L3,
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
    switch (io_.robot_state_) {
      case TARGET_ACQUISITION:
        io_.reef_target = reefPose(io_.target_column);
        if (io_.reef_target.isPresent()) {
          SwerveDrivetrain.getInstance().setTargetPose(io_.reef_target.get());
          if (FieldRegions.REEF_ENTER.contains(PoseEstimator.getInstance().getRobotPose())) {
            io_.robot_state_ = RobotState.APPROACHING_TARGET;
          }
        }
        break;
      case APPROACHING_TARGET:
        if (Util.epislonEquals(
            PoseEstimator.getInstance().getRobotPose().getRotation(),
            io_.reef_target.get().getRotation(),
            Constants.GameStateManager
                .REQUIRED_ROTATION_FOR_ELEVATOR)) { // move elevator once within rotation
          elevatorTargetSwitch();
        }
        if (SwerveDrivetrain.getInstance().atTractorBeamPose()
            && Elevator.getInstance().isElevatorAndArmAtTarget()) {
          // Once at final target, hand off control
          SwerveDrivetrain.getInstance().restoreDefaultDriveMode();
          io_.robot_state_ = RobotState.SCORING;
        }
        break;
      case SCORING:
        // wait until you leave the exit Circle
        if (!FieldRegions.REEF_EXIT.contains(PoseEstimator.getInstance().getRobotPose())) {
          io_.robot_state_ = RobotState.END;
        }
        break;
      case END:
        // clear saved vars and reset drive mode
        SwerveDrivetrain.getInstance().restoreDefaultDriveMode();
        if (Claw.getInstance().isCoralMode()) {
          Elevator.getInstance().setSpeedLimit(SpeedLimit.CORAL);
          Elevator.getInstance().setTarget(Target.STOW);
        } else {
          Elevator.getInstance().setSpeedLimit(SpeedLimit.ALGAE);
          Elevator.getInstance().setTarget(Target.ALGAE_STOW);
        }
        io_.robot_state_ = RobotState.TELEOP_CONTROL;
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
    SmartDashboard.putString(
        "Subsystems/GameStateManager/Robot State", io_.robot_state_.toString());
    SmartDashboard.putString(
        "Subsystems/GameStateManager/Target Colum", io_.target_column.toString());
    SmartDashboard.putString(
        "Subsystems/GameStateManager/Saved Colum", io_.saved_target_column.toString());
    SmartDashboard.putString(
        "Subsystems/GameStateManager/Target Level", io_.scoring_target.toString());
    SmartDashboard.putString(
        "Subsystems/GameStateManager/Saved Target Level", io_.saved_scoring_target.toString());
    SmartDashboard.putBoolean(
        "Subsystems/GameStateManager/Ready to Score",
        (SwerveDrivetrain.getInstance().atTractorBeamPose()
            && Elevator.getInstance().isElevatorAndArmAtTarget()
            && io_.robot_state_ == RobotState.SCORING));
    if (io_.reef_target.isPresent()) {
      reef_target_publisher.set(io_.reef_target.get());
    } else {
      reef_target_publisher.set(new Pose2d());
    }
  }

  /**
   * @param target_score new target
   * @param save weather or not to save the target
   */
  public void setScoringColum(Column col, boolean save) {
    io_.target_column = col;
    if (save) {
      io_.saved_target_column = col;
    }
  }

  /**
   * @param target_score new target
   * @param save weather or not to save the target
   */
  public void setScoringTarget(ReefScoringTarget target, boolean save) {
    io_.scoring_target = target;
    if (save) {
      io_.saved_scoring_target = target;
    }
  }

  public void setScoringObj(Column col, ReefScoringTarget target, boolean save) {
    setScoringColum(col, save);
    setScoringTarget(target, save);
  }

  public static Command setScoringCommand(Column col, ReefScoringTarget target) {
    return Commands.runOnce(() -> getInstance().setScoringObj(col, target, false));
  }

  public ReefScoringTarget getSavedScoringTarget() {
    return io_.saved_scoring_target;
  }

  public Column getSavedScoringColum() {
    return io_.saved_target_column;
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
        if (column == Column.CENTER) {
          return Optional.of(
              FieldRegions.REGION_POSE_TABLE
                  .get(FieldRegions.REEF_REGIONS[i].getName())
                  .transformBy(ScoringPoses.L1_CORAL_OFFSET));
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
          // PoseEstimator.getInstance().reefPose()
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
    // TARGET_ACQUISITION to preserve
    // structure of the switch case)
    io_.robot_state_ = state;
  }

  public RobotState getRobotState() {
    return io_.robot_state_;
  }

  public void elevatorTargetSwitch() {
    switch (io_.scoring_target) {
      case L1:
        Elevator.getInstance().setTarget(Target.L1);
        break;
      case L2:
        Elevator.getInstance().setTarget(Target.L2);
        break;
      case L3:
        Elevator.getInstance().setTarget(Target.L3);
        break;
      case L4:
        Elevator.getInstance().setTarget(Target.L4);
        break;
      case ALGAE:
        if (io_.algae_level_high) {
          Elevator.getInstance().setTarget(Target.ALGAE_HIGH);
        } else {
          Elevator.getInstance().setTarget(Target.ALGAE_LOW);
        }
        break;
      case TURTLE:
      default:
        Elevator.getInstance().setTarget(Target.STOW);
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
    @Log.File public ReefScoringTarget scoring_target = ReefScoringTarget.TURTLE;
    @Log.File public ReefScoringTarget saved_scoring_target = ReefScoringTarget.L2;
    @Log.File public RobotState robot_state_ = RobotState.TELEOP_CONTROL;
    @Log.File public Optional<Pose2d> reef_target = Optional.empty();
    @Log.File public Column target_column = Column.LEFT;
    @Log.File public Column saved_target_column = Column.LEFT;

    @Log.File
    public boolean algae_level_high = false; // false is low level and true is the higher level
  }

  @Override
  public Logged getLoggingObject() {
    return io_;
  }
}
