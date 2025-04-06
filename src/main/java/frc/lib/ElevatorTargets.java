package frc.lib;

import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Constants;

public class ElevatorTargets {
  public enum TargetType {
    TEST(
        new TargetData(
            new Translation3d(0, 0, Constants.ElevatorConstants.ELEVATOR_HEIGHT_PIVOT_MIN), "TEST"),
        new Translation3d[0],
        new Translation3d[0]),
    SAFETY(
        new TargetData(new Translation3d(), "SAFETY"), new Translation3d[0], new Translation3d[0]),
    L4(new TargetData(new Translation3d(), "L4"), new Translation3d[0], new Translation3d[0]),
    L3(new TargetData(new Translation3d(), "L3"), new Translation3d[0], new Translation3d[0]),
    L2(new TargetData(new Translation3d(), "L2"), new Translation3d[0], new Translation3d[0]),
    L1(new TargetData(new Translation3d(), "L1"), new Translation3d[0], new Translation3d[0]),
    STATION(
        new TargetData(new Translation3d(), "STATION"), new Translation3d[0], new Translation3d[0]),
    CLIMB(new TargetData(new Translation3d(), "CLIMB"), new Translation3d[0], new Translation3d[0]),
    CORAL_INTAKE(
        new TargetData(new Translation3d(), "CORAL_INTAKE"),
        new Translation3d[0],
        new Translation3d[0]),
    CORAL_STOW(
        new TargetData(new Translation3d(), "CORAL_STOW"),
        new Translation3d[0],
        new Translation3d[0]),
    ALGAE_LOW(
        new TargetData(new Translation3d(), "ALGAE_LOW"),
        new Translation3d[0],
        new Translation3d[0]),
    ALGAE_HIGH(
        new TargetData(new Translation3d(), "ALGAE_HIGH"),
        new Translation3d[0],
        new Translation3d[0]),
    ALGAE_PROCESSOR(
        new TargetData(new Translation3d(), "ALGAE_PROCESSOR"),
        new Translation3d[0],
        new Translation3d[0]),
    BARGE(new TargetData(new Translation3d(), "BARGE"), new Translation3d[0], new Translation3d[0]),
    ALGAE_STOW(
        new TargetData(new Translation3d(), "ALGAE_STOW"),
        new Translation3d[0],
        new Translation3d[0]);

    TargetType(TargetData target, Translation3d[] enter, Translation3d[] exit) {
      this.target = target;
      this.enter_trj = enter;
      this.exit_trj = exit;
    }

    private TargetData target;
    private Translation3d[] enter_trj;
    private Translation3d[] exit_trj;

    public Translation3d[] getEnterTrj() {
      return enter_trj;
    }

    public Translation3d[] getExitTrj() {
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
