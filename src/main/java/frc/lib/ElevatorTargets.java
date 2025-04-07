package frc.lib;

import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Constants;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class ElevatorTargets {
  public enum TargetType {
    TEST(
        new TargetData(
            new Translation3d(0, 0, Constants.ElevatorConstants.ELEVATOR_HEIGHT_PIVOT_MIN), "TEST"),
        Arrays.asList(new Translation3d(0.2, 0, 1)),
        Arrays.asList()),
    SAFETY(new TargetData(new Translation3d(), "SAFETY"), Arrays.asList(), Arrays.asList()),
    L4(
        new TargetData(
            new Translation3d(
                Constants.DrivetrainConstants.CENTER_OFFSET_X - 0.1,
                0,
                FieldConstants.ReefHeight.L4.HEIGHT),
            "L4"),
        Arrays.asList(),
        Arrays.asList()),
    L3(
        new TargetData(
            new Translation3d(
                Constants.DrivetrainConstants.CENTER_OFFSET_X - 0.1,
                0,
                FieldConstants.ReefHeight.L3.HEIGHT),
            "L3"),
        Arrays.asList(),
        Arrays.asList()),
    L2(
        new TargetData(
            new Translation3d(
                Constants.DrivetrainConstants.CENTER_OFFSET_X - 0.1,
                0,
                FieldConstants.ReefHeight.L2.HEIGHT),
            "L2"),
        Arrays.asList(),
        Arrays.asList()),
    L1(
        new TargetData(
            new Translation3d(
                Constants.DrivetrainConstants.CENTER_OFFSET_X - 0.1,
                0,
                FieldConstants.ReefHeight.L1.HEIGHT),
            "L1"),
        Arrays.asList(),
        Arrays.asList()),
    STATION(new TargetData(new Translation3d(), "STATION"), Arrays.asList(), Arrays.asList()),
    CLIMB(new TargetData(new Translation3d(), "CLIMB"), Arrays.asList(), Arrays.asList()),
    CORAL_INTAKE(
        new TargetData(
            new Translation3d(Constants.DrivetrainConstants.CENTER_OFFSET_X - 0.1, 0, 1),
            "CORAL_INTAKE"),
        Arrays.asList(),
        Arrays.asList()),
    CORAL_STOW(
        new TargetData(CORAL_INTAKE.getTarget().getTranslation(), "CORAL_STOW"),
        Arrays.asList(),
        Arrays.asList()),
    ALGAE_LOW(new TargetData(new Translation3d(), "ALGAE_LOW"), Arrays.asList(), Arrays.asList()),
    ALGAE_HIGH(new TargetData(new Translation3d(), "ALGAE_HIGH"), Arrays.asList(), Arrays.asList()),
    ALGAE_PROCESSOR(
        new TargetData(new Translation3d(), "ALGAE_PROCESSOR"), Arrays.asList(), Arrays.asList()),
    BARGE(new TargetData(new Translation3d(), "BARGE"), Arrays.asList(), Arrays.asList()),
    ALGAE_STOW(new TargetData(new Translation3d(), "ALGAE_STOW"), Arrays.asList(), Arrays.asList());

    TargetType(TargetData target, List<Translation3d> enter, List<Translation3d> exit) {
      this.target = target;
      this.enter_trj = new ArrayList<Translation3d>(enter);
      this.exit_trj = new ArrayList<Translation3d>(exit);
    }

    private TargetData target;
    private ArrayList<Translation3d> enter_trj;
    private ArrayList<Translation3d> exit_trj;

    public ArrayList<Translation3d> getEnterTrj() {
      return enter_trj;
    }

    public ArrayList<Translation3d> getExitTrj() {
      return exit_trj;
    }

    public TargetData getTarget() {
      return target;
    }

    public void offsetY(double offset) {
      target.offsetZ(offset);
    }

    public void resetYOffset() {
      target.resetZOffset();
    }

    public void offsetX(double offset) {
      target.offsetX(offset);
    }

    public void resetXOffset() {
      target.resetXOffset();
    }

    public TargetData getLoggingObject() {
      return target;
    }
  }
}
