// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.net.WebServer;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import edu.wpi.first.wpilibj.Filesystem;
import frc.mw_lib.subsystem.Subsystem;
import frc.robot.Constants.ReefControlsConstants;
import java.nio.file.Paths;
import java.util.Arrays;
import monologue.Logged;

public class ReefControls extends Subsystem {
  // Singleton pattern
  private static ReefControls reef_controls_instance_ = null;

  public static ReefControls getInstance() {
    if (reef_controls_instance_ == null) {
      reef_controls_instance_ = new ReefControls();
    }
    return reef_controls_instance_;
  }

  class ReefControlsIOInputs {
    public int[] selectedLevel = new int[] {}; // 0 = L2, 1 = L3, 2 = L4
    public int[] level1State = new int[] {}; // Count
    public int[] level2State = new int[] {}; // Bitfield
    public int[] level3State = new int[] {}; // Bitfield
    public int[] level4State = new int[] {}; // Bitfield
    public int[] algaeState = new int[] {}; // Bitfield
    public boolean[] coopState = new boolean[] {}; // Boolean
  }

  ReefControlsIOInputs reefStateInputs = new ReefControlsIOInputs();

  private final IntegerSubscriber selectedLevelIn;
  private final IntegerSubscriber l1StateIn;
  private final IntegerSubscriber l2StateIn;
  private final IntegerSubscriber l3StateIn;
  private final IntegerSubscriber l4StateIn;
  private final IntegerSubscriber algaeStateIn;
  private final BooleanSubscriber coopStateIn;

  private final IntegerPublisher selectedLevelOut;
  private final IntegerPublisher l1StateOut;
  private final IntegerPublisher l2StateOut;
  private final IntegerPublisher l3StateOut;
  private final IntegerPublisher l4StateOut;
  private final IntegerPublisher algaeStateOut;
  private final BooleanPublisher coopStateOut;
  private final BooleanPublisher isElimsOut;

  /** Class Members */
  private ReeefControlsPeriodicIo io_;

  private ReefControls() {
    // Create io object first in subsystem configuration
    io_ = new ReeefControlsPeriodicIo();

    // Create subscribers
    var inputTable = NetworkTableInstance.getDefault().getTable(ReefControlsConstants.toRobotTable);
    selectedLevelIn =
        inputTable
            .getIntegerTopic(ReefControlsConstants.selectedLevelTopicName)
            .subscribe(0, PubSubOption.keepDuplicates(true));
    l1StateIn =
        inputTable
            .getIntegerTopic(ReefControlsConstants.l1TopicName)
            .subscribe(0, PubSubOption.keepDuplicates(true));
    l2StateIn =
        inputTable
            .getIntegerTopic(ReefControlsConstants.l2TopicName)
            .subscribe(0, PubSubOption.keepDuplicates(true));
    l3StateIn =
        inputTable
            .getIntegerTopic(ReefControlsConstants.l3TopicName)
            .subscribe(0, PubSubOption.keepDuplicates(true));
    l4StateIn =
        inputTable
            .getIntegerTopic(ReefControlsConstants.l4TopicName)
            .subscribe(0, PubSubOption.keepDuplicates(true));
    algaeStateIn =
        inputTable
            .getIntegerTopic(ReefControlsConstants.algaeTopicName)
            .subscribe(0, PubSubOption.keepDuplicates(true));
    coopStateIn =
        inputTable
            .getBooleanTopic(ReefControlsConstants.coopTopicName)
            .subscribe(false, PubSubOption.keepDuplicates(true));

    // Create publishers
    var outputTable =
        NetworkTableInstance.getDefault().getTable(ReefControlsConstants.toDashboardTable);
    selectedLevelOut =
        outputTable.getIntegerTopic(ReefControlsConstants.selectedLevelTopicName).publish();
    l1StateOut = outputTable.getIntegerTopic(ReefControlsConstants.l1TopicName).publish();
    l2StateOut = outputTable.getIntegerTopic(ReefControlsConstants.l2TopicName).publish();
    l3StateOut = outputTable.getIntegerTopic(ReefControlsConstants.l3TopicName).publish();
    l4StateOut = outputTable.getIntegerTopic(ReefControlsConstants.l4TopicName).publish();
    algaeStateOut = outputTable.getIntegerTopic(ReefControlsConstants.algaeTopicName).publish();
    coopStateOut = outputTable.getBooleanTopic(ReefControlsConstants.coopTopicName).publish();
    isElimsOut = outputTable.getBooleanTopic(ReefControlsConstants.isElimsTopicName).publish();

    // Start web server
    WebServer.start(
        ReefControlsConstants.port,
        Paths.get(Filesystem.getDeployDirectory().getAbsolutePath().toString(), "reefcontrols")
            .toString());

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
    reefStateInputs.selectedLevel =
        selectedLevelIn.readQueue().length > 0
            ? new int[] {(int) selectedLevelIn.get()}
            : new int[] {};
    reefStateInputs.level1State =
        l1StateIn.readQueue().length > 0 ? new int[] {(int) l1StateIn.get()} : new int[] {};
    reefStateInputs.level2State =
        l2StateIn.readQueue().length > 0 ? new int[] {(int) l2StateIn.get()} : new int[] {};
    reefStateInputs.level3State =
        l3StateIn.readQueue().length > 0 ? new int[] {(int) l3StateIn.get()} : new int[] {};
    reefStateInputs.level4State =
        l4StateIn.readQueue().length > 0 ? new int[] {(int) l4StateIn.get()} : new int[] {};
    reefStateInputs.algaeState =
        algaeStateIn.readQueue().length > 0 ? new int[] {(int) algaeStateIn.get()} : new int[] {};
    reefStateInputs.coopState =
        coopStateIn.readQueue().length > 0 ? new boolean[] {coopStateIn.get()} : new boolean[] {};
  }

  /**
   * Inside this function, all of the LOGIC should compute updates to output variables in the
   * PeriodicIO class defined below. There should be no calls to read from sensors or write to
   * actuators in this function.
   */
  @Override
  public void updateLogic(double timestamp) {
    // Update reef state from inputs
    if (reefStateInputs.level1State.length > 0) {
      io_.reefState =
          new ReefState(
              io_.reefState.coral(), io_.reefState.algae(), reefStateInputs.level1State[0]);
    }
    final int[][] inputLevelStates =
        new int[][] {
          reefStateInputs.level2State, reefStateInputs.level3State, reefStateInputs.level4State
        };
    for (int i = 0; i < 3; i++) {
      if (inputLevelStates[i].length > 0) {
        boolean[] levelState = new boolean[12];
        for (int j = 0; j < 12; j++) {
          levelState[j] = (inputLevelStates[i][0] & (1 << j)) != 0;
        }
        io_.reefState.coral()[i] = levelState;
      }
    }
    if (reefStateInputs.algaeState.length > 0) {
      boolean[] algae = new boolean[6];
      for (int j = 0; j < 6; j++) {
        algae[j] = (reefStateInputs.algaeState[0] & (1 << j)) != 0;
      }
      io_.reefState = new ReefState(io_.reefState.coral(), algae, io_.reefState.troughCount());
    }
    if (reefStateInputs.coopState.length > 0) {
      io_.coopState = reefStateInputs.coopState[0];
    }
    if (DriverStation.getMatchType() == MatchType.Elimination) {
      io_.coopState = false;
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
    // Publish state to dashboard
    if (reefStateInputs.selectedLevel.length > 0) {
      io_.selectedLevel = reefStateInputs.selectedLevel[0];
    }
    setSelectedLevel(io_.selectedLevel);
    setLevel1State(io_.reefState.troughCount());
    final int[] levelStates = new int[] {0, 0, 0};
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 12; j++) {
        if (io_.reefState.coral()[i][j]) {
          levelStates[i] |= 1 << j;
        }
      }
    }
    setLevel2State(levelStates[0]);
    setLevel3State(levelStates[1]);
    setLevel4State(levelStates[2]);
    int algaeState = 0;
    for (int i = 0; i < 6; i++) {
      if (io_.reefState.algae()[i]) {
        algaeState |= 1 << i;
      }
    }
    setAlgaeState(algaeState);
    setCoopState(io_.coopState);
    setElims(DriverStation.getMatchType() == MatchType.Elimination);
  }

  public void setSelectedLevel(int value) {
    selectedLevelOut.set(value);
  }

  public void setLevel1State(int value) {
    l1StateOut.set(value);
  }

  public void setLevel2State(int value) {
    l2StateOut.set(value);
  }

  public void setLevel3State(int value) {
    l3StateOut.set(value);
  }

  public void setLevel4State(int value) {
    l4StateOut.set(value);
  }

  public void setAlgaeState(int value) {
    algaeStateOut.set(value);
  }

  public void setCoopState(boolean value) {
    coopStateOut.set(value);
  }

  public void setElims(boolean isElims) {
    isElimsOut.set(isElims);
  }

  public class ReeefControlsPeriodicIo implements Logged {
    public ReefState reefState = ReefState.initial;
    public int selectedLevel = 0;
    public boolean coopState = false;
  }

  @Override
  public Logged getLoggingObject() {
    return io_;
  }

  private record ReefState(boolean[][] coral, boolean[] algae, int troughCount) {
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
      if (!(o instanceof ReefState reefState)) return false;
      return troughCount == reefState.troughCount
          && Arrays.equals(algae, reefState.algae)
          && Arrays.deepEquals(coral, reefState.coral);
    }

    @Override
    protected ReefState clone() {
      boolean[][] copy = new boolean[coral.length][coral[0].length];
      for (int i = 0; i < copy.length; i++) {
        copy[i] = Arrays.copyOf(coral[i], coral[i].length);
      }
      return new ReefState(copy, Arrays.copyOf(algae, algae.length), troughCount);
    }
  }
}
