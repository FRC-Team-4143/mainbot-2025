package frc.lib;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.lib.ElevatorKinematics.SolutionType;
import frc.mw_lib.geometry.spline.Waypoint;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class ElevatorTargets {
  private static final double IN_PERIMETER_X = Units.inchesToMeters(9.246 - 2);
  private static final double BRANCH_HEIGHT_BUMP = Units.inchesToMeters(7.315);

  public enum TargetType {
    SAFETY(new TargetData(new Waypoint(), "SAFETY"), Arrays.asList(), Arrays.asList()),
    L4(
        new TargetData(
            new Waypoint(
                Units.inchesToMeters(16.031),
                0,
                FieldConstants.ReefHeight.L4.HEIGHT + Units.inchesToMeters(5.297)),
            "L4"),
        Arrays.asList(new Waypoint(IN_PERIMETER_X, 0, Units.inchesToMeters(60))),
        Arrays.asList(new Waypoint(IN_PERIMETER_X, 0, Units.inchesToMeters(60)))),
    L3(
        new TargetData(
            new Waypoint(
                Units.inchesToMeters(17),
                0,
                FieldConstants.ReefHeight.L3.HEIGHT + BRANCH_HEIGHT_BUMP),
            "L3"),
        Arrays.asList(),
        Arrays.asList()),
    L2(
        new TargetData(
            new Waypoint(
                Units.inchesToMeters(17),
                0,
                FieldConstants.ReefHeight.L2.HEIGHT + BRANCH_HEIGHT_BUMP),
            "L2"),
        Arrays.asList(),
        Arrays.asList()),

    L1(
        new TargetData(
            new Waypoint(Units.inchesToMeters(15.616), 0, Units.inchesToMeters(27.688)), "L1"),
        Arrays.asList(new Waypoint(Units.inchesToMeters(0), 0, Units.inchesToMeters(27.688))),
        Arrays.asList(
            new Waypoint(
                Units.inchesToMeters(17),
                0,
                FieldConstants.ReefHeight.L2.HEIGHT + BRANCH_HEIGHT_BUMP))),
    STATION(
        new TargetData(
            new Waypoint(Units.inchesToMeters(-16.287), 0, Units.inchesToMeters(41.147)),
            "STATION"),
        Arrays.asList(new Waypoint(0, 0, Units.inchesToMeters(41.147))),
        Arrays.asList(new Waypoint(0, 0, Units.inchesToMeters(41.147)))),
    CLIMB(
        new TargetData(
            new Waypoint(Units.inchesToMeters(-8.848), 0, Units.inchesToMeters(47.579)), "CLIMB"),
        Arrays.asList(new Waypoint(Units.inchesToMeters(1.956), 0, Units.inchesToMeters(44.809))),
        Arrays.asList(new Waypoint(Units.inchesToMeters(1.956), 0, Units.inchesToMeters(44.809))),
        SolutionType.ABOVE_PIVOT),
    CORAL_INTAKE(
        new TargetData(
            new Waypoint(Units.inchesToMeters(-8.077), 0, Units.inchesToMeters(14.662)),
            "CORAL_INTAKE"),
        Arrays.asList(),
        Arrays.asList()),
    CORAL_STOW(
        new TargetData(CORAL_INTAKE.getTarget().getTranslation(), "CORAL_STOW"),
        Arrays.asList(),
        Arrays.asList()),
    ALGAE_LOW(
        new TargetData(new Waypoint(0.42189, 0, 1.262395), "ALGAE_LOW"),
        Arrays.asList(),
        Arrays.asList()),
    ALGAE_HIGH(
        new TargetData(new Waypoint(0.42189, 0, 1.262395), "ALGAE_HIGH"),
        Arrays.asList(),
        Arrays.asList()),
    ALGAE_PROCESSOR(
        new TargetData(new Waypoint(0.33, 0, 0.5), "ALGAE_PROCESSOR"),
        Arrays.asList(),
        Arrays.asList()),
    BARGE(
        new TargetData(
            new Waypoint(Units.inchesToMeters(15.637), 0, Units.inchesToMeters(86.665)), "BARGE"),
        Arrays.asList(),
        Arrays.asList(),
        SolutionType.ABOVE_PIVOT),
    ALGAE_STOW(
        new TargetData(
            new Waypoint(Units.inchesToMeters(16.593), 0, FieldConstants.ReefHeight.L2.HEIGHT),
            "ALGAE_STOW"),
        Arrays.asList(),
        Arrays.asList());

    TargetType(TargetData target, List<Waypoint> enter, List<Waypoint> exit) {
      this.target = target;
      this.enter_trj = new ArrayList<Waypoint>(enter);
      this.exit_trj = new ArrayList<Waypoint>(exit);
      this.soultion_type = SolutionType.BELOW_PIVOT;
    }

    TargetType(TargetData target, List<Waypoint> enter, List<Waypoint> exit, SolutionType es) {
      this.target = target;
      this.enter_trj = new ArrayList<Waypoint>(enter);
      this.exit_trj = new ArrayList<Waypoint>(exit);
      this.soultion_type = es;
    }

    private TargetData target;
    private SolutionType soultion_type;
    private ArrayList<Waypoint> enter_trj;
    private ArrayList<Waypoint> exit_trj;

    public SolutionType getJointSpaceSolution() {
      return soultion_type;
    }

    public ArrayList<Waypoint> getEnterTrj() {
      return enter_trj;
    }

    public ArrayList<Waypoint> getExitTrj() {
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
