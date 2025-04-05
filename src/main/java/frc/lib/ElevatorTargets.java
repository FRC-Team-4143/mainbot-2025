package frc.lib;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.lib.TargetData.ControlType;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorTargets {
  public enum TargetType {
    SAFETY(
        new TargetData(
            ElevatorConstants
                .ELEVATOR_HEIGHT_PIVOT_SAFETY, // height is ignored for this special target
            Units.degreesToRadians(90),
            ControlType.PIVOT,
            "SAFETY"),
        new Translation2d[0],
        new Translation2d[0]),
    L4(
        new TargetData(
            FieldConstants.ReefHeight.L4.HEIGHT + Units.inchesToMeters(11),
            Units.degreesToRadians(-180 - (180 - 145.3)),
            ControlType.EFFECTOR,
            "L4"),
        new Translation2d[0],
        new Translation2d[0]),
    L3(
        new TargetData(
            FieldConstants.ReefHeight.L3.HEIGHT + Units.inchesToMeters(8),
            Units.degreesToRadians(-180 - (180 - 125)),
            ControlType.EFFECTOR,
            "L3"),
        new Translation2d[0],
        new Translation2d[0]),
    L2(
        new TargetData(
            FieldConstants.ReefHeight.L2.HEIGHT + Units.inchesToMeters(8),
            Units.degreesToRadians(-180 - (180 - 125)),
            ControlType.EFFECTOR,
            "L2"),
        new Translation2d[0],
        new Translation2d[0]),
    L1(
        new TargetData(
            FieldConstants.ReefHeight.L2.HEIGHT + Units.inchesToMeters(8),
            Units.degreesToRadians(-180 - (180 - 125)),
            ControlType.EFFECTOR,
            "L1"),
        new Translation2d[0],
        new Translation2d[0]),
    STATION(
        new TargetData(
            0.8468 + Units.inchesToMeters(0), (-1.027767), ControlType.EFFECTOR, "STATION"),
        new Translation2d[0],
        new Translation2d[0]),
    CLIMB(
        new TargetData(
            ElevatorConstants.ELEVATOR_HEIGHT_PIVOT_MIN + Units.inchesToMeters(0),
            Units.degreesToRadians(20),
            ControlType.PIVOT,
            "CLIMB"),
        new Translation2d[0],
        new Translation2d[0]),
    CORAL_INTAKE(
        new TargetData(
            0.7891225351316538 + Units.inchesToMeters(1),
            Units.degreesToRadians(-115),
            ControlType.PIVOT,
            "CORAL_INTAKE"),
        new Translation2d[0],
        new Translation2d[0]),
    CORAL_STOW(
        new TargetData(
            ElevatorConstants.ELEVATOR_HEIGHT_PIVOT_MIN,
            Units.degreesToRadians(-270),
            ControlType.PIVOT,
            "CORAL_STOW"),
        new Translation2d[0],
        new Translation2d[0]),
    ALGAE_LOW(
        new TargetData(
            0.9702231159054557 + Units.inchesToMeters(0),
            Units.degreesToRadians(-180 - (180 - 149)),
            ControlType.EFFECTOR,
            "ALGAE_LOW"),
        new Translation2d[0],
        new Translation2d[0]),
    ALGAE_HIGH(
        new TargetData(
            1.2535345791562702 + Units.inchesToMeters(0),
            Units.degreesToRadians(-180 - (180 - 130.79)),
            ControlType.EFFECTOR,
            "ALGAE_HIGH"),
        new Translation2d[0],
        new Translation2d[0]),
    ALGAE_PROCESSOR(
        new TargetData(
            FieldConstants.ReefHeight.L3.HEIGHT - Units.inchesToMeters(-18),
            Units.degreesToRadians(-55),
            ControlType.EFFECTOR,
            "ALGAE_PROCESSOR"),
        new Translation2d[0],
        new Translation2d[0]),
    BARGE(
        new TargetData(
            2.159 + Units.inchesToMeters(0),
            Units.degreesToRadians(-270),
            ControlType.EFFECTOR,
            "BARGE"),
        new Translation2d[0],
        new Translation2d[0]),
    ALGAE_STOW(
        new TargetData(
            ElevatorConstants.ELEVATOR_HEIGHT_PIVOT_MIN + Units.inchesToMeters(0),
            Units.degreesToRadians(-270),
            ControlType.PIVOT,
            "ALGAE_STOW"),
        new Translation2d[0],
        new Translation2d[0]);

    TargetType(TargetData target, Translation2d[] enter, Translation2d[] exit) {
      this.target = target;
      this.enter_trj = enter;
      this.exit_trj = exit;
    }

    private TargetData target;
    private Translation2d[] enter_trj;
    private Translation2d[] exit_trj;

    public Translation2d[] getEnterTarget() {
      return enter_trj;
    }

    public Translation2d[] getExitTarget() {
      return exit_trj;
    }

    public TargetData getTarget() {
      return target;
    }

    // Height Methods
    /**
     * Adjusts the height offset by the supplied increment
     *
     * @param offset
     */
    public void offsetHeight(double offset) {
      target.offsetHeight(offset);
    }

    /** Reset the height offset to 0 */
    public void resetHeightOffset() {
      target.resetHeightOffset();
    }

    // Angle Methods
    /**
     * Adjusts the angle offset by the supplied increment
     *
     * @param offset
     */
    public void offsetAngle(double offset) {
      target.offsetAngle(offset);
    }

    /** Resets the angle offset to 0 */
    public void resetAngleOffset() {
      target.resetAngleOffset();
    }

    /**
     * Returns the control type associated with the target
     *
     * @return stored control type
     */
    public ControlType getControlType() {
      return target.type_;
    }

    public TargetData getLoggingObject() {
      return target;
    }
  }
}
