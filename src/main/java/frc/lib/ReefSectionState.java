package frc.lib;

import java.util.ArrayList;

import monologue.Logged;
import monologue.Annotations.Log;

public class ReefSectionState implements Logged {

    @Log.File public boolean[] left_column = { false, false, false };
    @Log.File public boolean[] algae = { false, false };
    @Log.File public boolean[] right_column = { false, false, false };

    public static ReefSectionState averageReefSections(ArrayList<ReefSectionState> givenStates, double threshold) {
        ReefSectionState final_reef_section_state = new ReefSectionState();
        double[] left_column_count = { 0, 0, 0 },
                right_column_count = { 0, 0, 0 },
                algae_count = { 0, 0 };
        for (ReefSectionState current_state : givenStates) {
            for (int i = 0; i < left_column_count.length; i++) {
                if (current_state.left_column[i]) {
                    left_column_count[i] += 1;
                }
                if (current_state.right_column[1]) {
                    right_column_count[i] += 1;
                }
            }
            for (int j = 0; j < algae_count.length; j++) {
                if (current_state.algae[j]) {
                    algae_count[j] += 1;
                }
            }
        }

        for (int i = 0; i < left_column_count.length; i++) {
            final_reef_section_state.left_column[i] = (left_column_count[i]
                    / left_column_count.length) < threshold;
            final_reef_section_state.right_column[i] = (right_column_count[i]
                    / right_column_count.length) < threshold;
        }

        for (int j = 0; j < algae_count.length; j++) {
            final_reef_section_state.algae[j] = (algae_count[j]
                    / algae_count.length) < threshold;

        }

        return final_reef_section_state;

    }
}
