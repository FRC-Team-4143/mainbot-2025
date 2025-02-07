package frc.robot;

import monologue.Annotations.Log;
import frc.lib.ReefSectionState;
import monologue.Logged;

public class GameStateManager implements Logged {

    @Log.File
    private ScoringTarget current_scoring_target_ = ScoringTarget.TURTLE;
    @Log.File
    private ReefSectionState current_reef_section_state_;
    @Log.File
    private RobotState current_robot_state_;

    public GameStateManager() {

    }

    public enum RobotState {
        TARGET_ACQUISITION,
        APPROACHING_TARGET,
        SCORING,
    }

    public enum ScoringTarget {
        TURTLE,
        REEF_L1,
        REEF_L2,
        REEF_L3,
        REEF_L4,
        REEF_ALGAE
    }

    public void robotStateSwitch() {
        switch (current_robot_state_) {
            case TARGET_ACQUISITION:

                break;
            case APPROACHING_TARGET:

                break;
            case SCORING:
            
                break;
        }
    }

}
