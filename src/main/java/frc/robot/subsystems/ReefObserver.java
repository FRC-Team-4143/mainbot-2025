// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.AllianceFlipUtil;
import frc.lib.FieldConstants;
import frc.lib.FieldConstants.ReefHeight;
import frc.mw_lib.subsystem.Subsystem;
import frc.mw_lib.util.GeomUtil;
import frc.robot.Constants;
import frc.robot.Constants.ReefControlsConstants;
import frc.robot.subsystems.GameStateManager.Column;
import frc.robot.subsystems.GameStateManager.GameStateTarget;
import frc.robot.subsystems.GameStateManager.ReefScoringTarget;
import java.nio.file.Paths;
import java.util.Arrays;
import java.util.HashSet;
import java.util.Optional;
import java.util.Set;
import monologue.Logged;

public class ReefObserver extends Subsystem {
  // Singleton pattern
  private static ReefObserver reef_controls_instance_ = null;

  public static ReefObserver getInstance() {
    if (reef_controls_instance_ == null) {
      reef_controls_instance_ = new ReefObserver();
    }
    return reef_controls_instance_;
  }

  class ReefControlInputs {
    public int[] selected_level = new int[] {}; // 0 = L2, 1 = L3, 2 = L4
    public int[] level_1_state = new int[] {}; // Count
    public int[] level_2_state = new int[] {}; // Bitfield
    public int[] level_3_state = new int[] {}; // Bitfield
    public int[] level_4_state = new int[] {}; // Bitfield
    public int[] algae_state = new int[] {}; // Bitfield
    public boolean[] coop_state = new boolean[] {}; // Boolean
    public boolean[] rp_focus_state = new boolean[] {}; // Boolean
  }

  ReefControlInputs reef_control_inputs_ = new ReefControlInputs();

  private final IntegerSubscriber selected_level_in_;
  private final IntegerSubscriber l1_state_in_;
  private final IntegerSubscriber l2_state_in_;
  private final IntegerSubscriber l3_state_in_;
  private final IntegerSubscriber l4_state_in_;
  private final IntegerSubscriber algae_state_in_;
  private final BooleanSubscriber coop_state_in_;
  private final BooleanSubscriber rp_focus_state_in_;

  private final IntegerPublisher selected_level_out_;
  private final IntegerPublisher l1_state_out_;
  private final IntegerPublisher l2_state_out_;
  private final IntegerPublisher l3_state_out_;
  private final IntegerPublisher l4_state_out_;
  private final IntegerPublisher algae_state_out_;
  private final BooleanPublisher coop_state_out_;
  private final BooleanPublisher is_elims_out_;
  private final BooleanPublisher rp_focus_state_out_;

  private final StructArrayPublisher<Pose3d> coral_pub_;
  private final StructArrayPublisher<Translation3d> algae_pub_;

  /** Class Members */
  private ReefObserverPeriodicIo io_;

  private ReefObserver() {
    // Create io object first in subsystem configuration
    io_ = new ReefObserverPeriodicIo();

    // Create subscribers
    var inputTable =
        NetworkTableInstance.getDefault().getTable(ReefControlsConstants.TO_ROBOT_TABLE);
    // Create subscribers
    var input_table_ =
        NetworkTableInstance.getDefault().getTable(ReefControlsConstants.TO_ROBOT_TABLE);
    selected_level_in_ =
        input_table_
            .getIntegerTopic(ReefControlsConstants.SELECTED_LEVEL_TOPIC_NAME)
            .subscribe(0, PubSubOption.keepDuplicates(true));
    l1_state_in_ =
        input_table_
            .getIntegerTopic(ReefControlsConstants.L1_TOPIC_NAME)
            .subscribe(0, PubSubOption.keepDuplicates(true));
    l2_state_in_ =
        input_table_
            .getIntegerTopic(ReefControlsConstants.L2_TOPIC_NAME)
            .subscribe(0, PubSubOption.keepDuplicates(true));
    l3_state_in_ =
        input_table_
            .getIntegerTopic(ReefControlsConstants.L3_TOPIC_NAME)
            .subscribe(0, PubSubOption.keepDuplicates(true));
    l4_state_in_ =
        input_table_
            .getIntegerTopic(ReefControlsConstants.L4_TOPIC_NAME)
            .subscribe(0, PubSubOption.keepDuplicates(true));
    algae_state_in_ =
        input_table_
            .getIntegerTopic(ReefControlsConstants.ALGAE_TOPIC_NAME)
            .subscribe(0, PubSubOption.keepDuplicates(true));
    coop_state_in_ =
        input_table_
            .getBooleanTopic(ReefControlsConstants.COOP_TOPIC_NAME)
            .subscribe(false, PubSubOption.keepDuplicates(true));
    rp_focus_state_in_ =
        input_table_
            .getBooleanTopic(ReefControlsConstants.RP_FOCUS_TOPIC_NAME)
            .subscribe(false, PubSubOption.keepDuplicates(true));

    // Create publishers
    var output_table_ =
        NetworkTableInstance.getDefault().getTable(ReefControlsConstants.TO_DASHBOARD_TABLE);
    selected_level_out_ =
        output_table_
            .getIntegerTopic(ReefControlsConstants.SELECTED_LEVEL_TOPIC_NAME)
            .publish(PubSubOption.keepDuplicates(true));
    l1_state_out_ =
        output_table_
            .getIntegerTopic(ReefControlsConstants.L1_TOPIC_NAME)
            .publish(PubSubOption.keepDuplicates(true));
    l2_state_out_ =
        output_table_
            .getIntegerTopic(ReefControlsConstants.L2_TOPIC_NAME)
            .publish(PubSubOption.keepDuplicates(true));
    l3_state_out_ =
        output_table_
            .getIntegerTopic(ReefControlsConstants.L3_TOPIC_NAME)
            .publish(PubSubOption.keepDuplicates(true));
    l4_state_out_ =
        output_table_
            .getIntegerTopic(ReefControlsConstants.L4_TOPIC_NAME)
            .publish(PubSubOption.keepDuplicates(true));
    algae_state_out_ =
        output_table_
            .getIntegerTopic(ReefControlsConstants.ALGAE_TOPIC_NAME)
            .publish(PubSubOption.keepDuplicates(true));
    coop_state_out_ =
        output_table_
            .getBooleanTopic(ReefControlsConstants.COOP_TOPIC_NAME)
            .publish(PubSubOption.keepDuplicates(true));
    is_elims_out_ =
        output_table_
            .getBooleanTopic(ReefControlsConstants.IS_ELIMS_TOPIC_NAME)
            .publish(PubSubOption.keepDuplicates(true));
    rp_focus_state_out_ =
        output_table_
            .getBooleanTopic(ReefControlsConstants.RP_FOCUS_TOPIC_NAME)
            .publish(PubSubOption.keepDuplicates(true));

    // Start web server
    WebServer.start(
        ReefControlsConstants.PORT,
        Paths.get(Filesystem.getDeployDirectory().getAbsolutePath().toString(), "reefcontrols")
            .toString());

    coral_pub_ =
        NetworkTableInstance.getDefault()
            .getStructArrayTopic("Reef Observer/Coral", Pose3d.struct)
            .publish();

    algae_pub_ =
        NetworkTableInstance.getDefault()
            .getStructArrayTopic("Reef Observer/Algae", Translation3d.struct)
            .publish();

    // Call reset last in subsystem configuration
    reset();
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
  public void readPeriodicInputs(double timestamp) {
    reef_control_inputs_.selected_level =
        selected_level_in_.readQueue().length > 0
            ? new int[] {(int) selected_level_in_.get()}
            : new int[] {};
    reef_control_inputs_.level_1_state =
        l1_state_in_.readQueue().length > 0 ? new int[] {(int) l1_state_in_.get()} : new int[] {};
    reef_control_inputs_.level_2_state =
        l2_state_in_.readQueue().length > 0 ? new int[] {(int) l2_state_in_.get()} : new int[] {};
    reef_control_inputs_.level_3_state =
        l3_state_in_.readQueue().length > 0 ? new int[] {(int) l3_state_in_.get()} : new int[] {};
    reef_control_inputs_.level_4_state =
        l4_state_in_.readQueue().length > 0 ? new int[] {(int) l4_state_in_.get()} : new int[] {};
    reef_control_inputs_.algae_state =
        algae_state_in_.readQueue().length > 0
            ? new int[] {(int) algae_state_in_.get()}
            : new int[] {};
    reef_control_inputs_.coop_state =
        coop_state_in_.readQueue().length > 0
            ? new boolean[] {coop_state_in_.get()}
            : new boolean[] {};
    reef_control_inputs_.rp_focus_state =
        rp_focus_state_in_.readQueue().length > 0
            ? new boolean[] {rp_focus_state_in_.get()}
            : new boolean[] {};
  }

  /**
   * Inside this function, all of the LOGIC should compute updates to output variables in the
   * PeriodicIO class defined below. There should be no calls to read from sensors or write to
   * actuators in this function.
   */
  @Override
  public void updateLogic(double timestamp) {
    // Update reef state from inputs
    if (reef_control_inputs_.level_1_state.length > 0) {
      io_.reef_state_ =
          new ReefState(
              io_.reef_state_.coral(),
              io_.reef_state_.algae(),
              reef_control_inputs_.level_1_state[0]);
    }
    final int[][] inputLevelStates =
        new int[][] {
          reef_control_inputs_.level_2_state,
          reef_control_inputs_.level_3_state,
          reef_control_inputs_.level_4_state
        };
    for (int i = 0; i < 3; i++) {
      if (inputLevelStates[i].length > 0) {
        boolean[] levelState = new boolean[12];
        for (int j = 0; j < 12; j++) {
          levelState[j] = (inputLevelStates[i][0] & (1 << j)) != 0;
        }
        io_.reef_state_.coral()[i] = levelState;
      }
    }
    if (reef_control_inputs_.algae_state.length > 0) {
      boolean[] algae = new boolean[6];
      for (int j = 0; j < 6; j++) {
        algae[j] = (reef_control_inputs_.algae_state[0] & (1 << j)) != 0;
      }
      io_.reef_state_ =
          new ReefState(io_.reef_state_.coral(), algae, io_.reef_state_.trough_count());
    }
    if (reef_control_inputs_.coop_state.length > 0) {
      io_.coop_state_ = reef_control_inputs_.coop_state[0];
    }
    if (DriverStation.getMatchType() == MatchType.Elimination) {
      io_.coop_state_ = false;
    }
    if (reef_control_inputs_.rp_focus_state.length > 0) {
      io_.rp_focus_state_ = reef_control_inputs_.rp_focus_state[0];
    }
  }

  /**
   * Inside this function actuator OUTPUTS should be updated from data contained in the PeriodicIO
   * class defined below. There should be little to no logic contained within this function, and no
   * sensors should be read.
   */
  @Override
  public void writePeriodicOutputs(double timestamp) {
    // Publish state to dashboard
    if (reef_control_inputs_.selected_level.length > 0) {
      io_.selected_level_ = reef_control_inputs_.selected_level[0];
    }
    setSelectedLevel(io_.selected_level_);
    setLevel1State(io_.reef_state_.trough_count());
    final int[] levelStates = new int[] {0, 0, 0};
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 12; j++) {
        if (io_.reef_state_.coral()[i][j]) {
          levelStates[i] |= 1 << j;
        }
      }
    }
    setLevel2State(levelStates[0]);
    setLevel3State(levelStates[1]);
    setLevel4State(levelStates[2]);
    int algae_state = 0;
    for (int i = 0; i < 6; i++) {
      if (io_.reef_state_.algae()[i]) {
        algae_state |= 1 << i;
      }
    }
    setAlgaeState(algae_state);
    setCoopState(io_.coop_state_);
    setElims(DriverStation.getMatchType() == MatchType.Elimination);
    setRpFocusState(io_.rp_focus_state_);
  }

  /**
   * Inside this function telemetry should be output to smartdashboard. The data should be collected
   * out of the PeriodicIO class instance defined below. There should be no sensor information read
   * in this function nor any outputs made to actuators within this function. Only publish to
   * smartdashboard here.
   */
  @Override
  public void outputTelemetry(double timestamp) {
    SmartDashboard.putBooleanArray("L2", getFace(0)[0]);
    SmartDashboard.putBooleanArray("L3", getFace(0)[1]);
    SmartDashboard.putBooleanArray("L4", getFace(0)[2]);
    SmartDashboard.putBoolean("RP Focus", io_.rp_focus_state_);

    if (!io_.reef_state_.equals(io_.previous_reef_state_)) {
      io_.previous_reef_state_ = io_.reef_state_.clone();
      publishReefState();
    }
  }

  public void publishReefState() {
    // Show state of reef in 3d
    Set<Pose3d> coral_poses = new HashSet<>();
    for (int level = 0; level < io_.reef_state_.coral().length; level++) {
      for (int j = 0; j < io_.reef_state_.coral()[0].length; j++) {
        if (!io_.reef_state_.coral()[level][j]) continue;
        Pose3d pose =
            FieldConstants.Reef.BRANCH_POSITIONS
                .get(j)
                .get(ReefHeight.fromLevel(level + 1))
                .transformBy(
                    GeomUtil.toTransform3d(
                        new Pose3d(
                            level == 2
                                ? new Translation3d(
                                    -Units.inchesToMeters(6.5), 0, Units.inchesToMeters(0.9))
                                : new Translation3d(
                                    Units.inchesToMeters(-4.5), 0.0, Units.inchesToMeters(-1.2)),
                            level == 2
                                ? new Rotation3d(0, Units.degreesToRadians(20), 0)
                                : Rotation3d.kZero)));
        coral_poses.add(
            (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue)
                ? pose
                : AllianceFlipUtil.apply(pose, FieldConstants.SYMMETRY_TYPE));
      }
    }
    coral_pub_.set(coral_poses.toArray(Pose3d[]::new));

    Set<Translation3d> algae_poses = new HashSet<>();
    for (int i = 0; i < 6; i++) {
      if (!io_.reef_state_.algae()[i]) continue;
      var first_branch_pose = FieldConstants.Reef.BRANCH_POSITIONS.get(i * 2).get(ReefHeight.L2);
      var second_branch_pose =
          FieldConstants.Reef.BRANCH_POSITIONS.get(i * 2 + 1).get(ReefHeight.L3);
      Translation3d pose =
          first_branch_pose
              .getTranslation()
              .interpolate(second_branch_pose.getTranslation(), 0.5)
              .plus(
                  new Translation3d(
                      -FieldConstants.ALGAE_DIAMETER / 3.0,
                      new Rotation3d(
                          0.0, -35.0 / 180.0 * Math.PI, first_branch_pose.getRotation().getZ())))
              .plus(
                  new Translation3d(
                      0.0,
                      0.0,
                      ((i % 2 == 0) ? second_branch_pose.getZ() - first_branch_pose.getZ() : 0.0)
                          + Units.inchesToMeters(-0.7)));
      algae_poses.add(
          (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue)
              ? pose
              : AllianceFlipUtil.apply(pose, FieldConstants.SYMMETRY_TYPE));
    }
    algae_pub_.set(algae_poses.toArray(Translation3d[]::new));
  }

  private void setSelectedLevel(int value) {
    selected_level_out_.set(value);
  }

  private void setLevel1State(int value) {
    l1_state_out_.set(value);
  }

  private void setLevel2State(int value) {
    l2_state_out_.set(value);
  }

  private void setLevel3State(int value) {
    l3_state_out_.set(value);
  }

  private void setLevel4State(int value) {
    l4_state_out_.set(value);
  }

  private void setAlgaeState(int value) {
    algae_state_out_.set(value);
  }

  private void setCoopState(boolean value) {
    coop_state_out_.set(value);
  }

  private void setElims(boolean isElims) {
    is_elims_out_.set(isElims);
  }

  private void setRpFocusState(boolean isRPFocus) {
    rp_focus_state_out_.set(isRPFocus);
  }

  public class ReefObserverPeriodicIo implements Logged {
    public ReefState reef_state_ = ReefState.initial;
    public ReefState previous_reef_state_ = null;
    public int selected_level_ = 0;
    public boolean coop_state_ = false;
    public boolean rp_focus_state_ = true;
  }

  @Override
  public Logged getLoggingObject() {
    return io_;
  }

  public boolean[][] getFace(int id) {
    boolean[][] grid = new boolean[3][2];
    int baseIndex = id * 2;

    grid[0][0] = !io_.reef_state_.coral[0][baseIndex + 1];
    grid[0][1] = !io_.reef_state_.coral[0][baseIndex];
    grid[1][0] = !io_.reef_state_.coral[1][baseIndex + 1];
    grid[1][1] = !io_.reef_state_.coral[1][baseIndex];
    grid[2][0] = !io_.reef_state_.coral[2][baseIndex + 1];
    grid[2][1] = !io_.reef_state_.coral[2][baseIndex];

    if (io_.reef_state_.algae[id]) {
      grid[1][0] = false;
      grid[1][1] = false;
      if (id % 2 != 0) {
        grid[0][0] = false;
        grid[0][1] = false;
      }
    }
    return grid;
  }

  public GameStateTarget findHighestPoint(boolean[][] grid) {
    for (int i = grid.length - 1; i >= 0; i--) {
      for (int j = 0; j < grid[i].length; j++) {
        if (grid[i][j]) {
          return GameStateManager.getInstance()
          .new GameStateTarget(Column.values()[j], ReefScoringTarget.values()[i]);
        }
      }
    }
    return GameStateManager.getInstance().new GameStateTarget(ReefScoringTarget.L1, Column.CENTER);
  }

  public GameStateTarget findHighestRankingPoint(boolean[][] grid) {
    for (int i = grid.length - 1; i >= 0; i--) {
      int rowCount = countTrues(io_.reef_state_.coral[i]);
      for (int j = 0; j < grid[i].length; j++) {
        if (grid[i][j] && rowCount < Constants.ReefControlsConstants.CORAL_NEEDED_FOR_RP) {
          return GameStateManager.getInstance()
          .new GameStateTarget(Column.values()[j], ReefScoringTarget.values()[i]);
        }
      }
    }
    return GameStateManager.getInstance().new GameStateTarget(ReefScoringTarget.L1, Column.CENTER);
  }

  public Optional<GameStateTarget> findNextTarget() {
    int currentFace = PoseEstimator.getInstance().reefPoseInt();
    if (currentFace != -1 && currentFace <= 7) {
      return Optional.empty();
    }
    boolean[][] grid = getFace(currentFace);
    if (io_.rp_focus_state_ == true) {
      return Optional.of(findHighestRankingPoint(grid));
    }
    return Optional.of(findHighestPoint(grid));
  }

  public int countTrues(boolean[] array) {
    int count = 0;
    for (int i = 0; i < array.length; i++) {
      if (array[i]) {
        count++;
      }
    }
    return count;
  }

  public void updateReefState(GameStateTarget target) {

    int col = 0;
    if (target.getReefScoringTarget() == ReefScoringTarget.ALGAE) {
      col = PoseEstimator.getInstance().reefPoseInt();
      if (col > 5) {
        col -= 6;
      }
      io_.reef_state_.algae[col] = false;

    } else {
      int row = 0;
      if (target.getColumn() == Column.LEFT) {
        col++;
      }
      switch (PoseEstimator.getInstance().reefPoseInt()) {
        case 1:
          col += 2;
          break;
        case 2:
          col += 4;
          break;
        case 3:
          col += 6;
          break;
        case 4:
          col += 8;
          break;
        case 5:
          col += 10;
          break;
        case 7:
          col += 2;
          break;
        case 8:
          col += 4;
          break;
        case 9:
          col += 6;
          break;
        case 10:
          col += 8;
          break;
        case 11:
          col += 10;
          break;
        default:
          break;
      }
      switch (target.getReefScoringTarget()) {
        case L2:
          row = 0;
          break;
        case L3:
          row = 1;
          break;
        case L4:
          row = 2;
          break;
        default:
          break;
      }
      io_.reef_state_.coral[row][col] = true;
    }
  }

  private record ReefState(boolean[][] coral, boolean[] algae, int trough_count) {

    public static final ReefState initial =
        new ReefState(
            new boolean[][] {
              new boolean[] {
                false, false, false, false, false, false, false, false, false, false, false, false
              },
              new boolean[] {
                false, false, false, false, false, false, false, false, false, false, false, false
              },
              new boolean[] {
                false, false, false, false, false, false, false, false, false, false, false, false
              }
            },
            new boolean[] {true, true, true, true, true, true},
            0);

    @Override
    public boolean equals(Object o) {
      if (!(o instanceof ReefState reef_state)) return false;
      return trough_count == reef_state.trough_count
          && Arrays.equals(algae, reef_state.algae)
          && Arrays.deepEquals(coral, reef_state.coral);
    }

    @Override
    protected ReefState clone() {
      boolean[][] copy = new boolean[coral.length][coral[0].length];
      for (int i = 0; i < copy.length; i++) {
        copy[i] = Arrays.copyOf(coral[i], coral[i].length);
      }
      return new ReefState(copy, Arrays.copyOf(algae, algae.length), trough_count);
    }
  }
}
