package frc.lib;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class ElevatorTargets {
  private static final double IN_PERIMETER_X = Units.inchesToMeters(9.246 - 2);
  private static final double BRANCH_HEIGHT_BUMP = Units.inchesToMeters(5.921);

  public enum TargetType {
    SAFETY(new TargetData(new Translation3d(), "SAFETY"), Arrays.asList(), Arrays.asList()),
    L4(
        new TargetData(
            new Translation3d(
                Units.inchesToMeters(16.031),
                0,
                FieldConstants.ReefHeight.L4.HEIGHT + Units.inchesToMeters(4.047)),
            "L4"),
        Arrays.asList(new Translation3d(IN_PERIMETER_X, 0, Units.inchesToMeters(68.109))),
        Arrays.asList(new Translation3d(IN_PERIMETER_X, 0, Units.inchesToMeters(68.109)))),
    L3(
        new TargetData(
            new Translation3d(
                Units.inchesToMeters(16.593),
                0,
                FieldConstants.ReefHeight.L3.HEIGHT + BRANCH_HEIGHT_BUMP),
            "L3"),
        Arrays.asList(new Translation3d(IN_PERIMETER_X, 0, FieldConstants.ReefHeight.L3.HEIGHT)),
        Arrays.asList(new Translation3d(IN_PERIMETER_X, 0, FieldConstants.ReefHeight.L3.HEIGHT))),
    L3_FAR(
        new TargetData(L3.getTarget().getTranslation(), "L3_FAR"),
        Arrays.asList(),
        Arrays.asList()),
    L2(
        new TargetData(
            new Translation3d(
                Units.inchesToMeters(16.593),
                0,
                FieldConstants.ReefHeight.L2.HEIGHT + BRANCH_HEIGHT_BUMP),
            "L2"),
        Arrays.asList(new Translation3d(IN_PERIMETER_X, 0, FieldConstants.ReefHeight.L2.HEIGHT)),
        Arrays.asList(new Translation3d(IN_PERIMETER_X, 0, FieldConstants.ReefHeight.L2.HEIGHT))),
    L2_FAR(
        new TargetData(L2.getTarget().getTranslation(), "L2_FAR"),
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
            new Translation3d(-0.206312455806457, 0, 0.36658133576115 + Units.inchesToMeters(2)),
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

    public void offSet(Translation3d t) {
      target.offSet(t);
    }

    public void resetOffsets() {
      target.resetOffsets();
    }

    public TargetData getLoggingObject() {
      return target;
    }
  }
}
