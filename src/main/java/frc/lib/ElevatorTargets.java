package frc.lib;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.lib.TargetData.ControlType;
import frc.robot.Constants.ElevatorConstants;
import java.util.Optional;

public class ElevatorTargets {
  public enum Target {
    L4(
        new TargetData(
            FieldConstants.ReefHeight.L4.HEIGHT + Units.inchesToMeters(11),
            Rotation2d.fromDegrees(145.3),
            ControlType.EFFECTOR),
        Optional.of(IntermediateTarget.L4_I),
        Optional.empty()),
    L3(
        new TargetData(
            FieldConstants.ReefHeight.L3.HEIGHT + Units.inchesToMeters(8),
            Rotation2d.fromDegrees(125),
            ControlType.EFFECTOR),
        Optional.of(IntermediateTarget.L3_I),
        Optional.empty()),
    L2(
        new TargetData(
            FieldConstants.ReefHeight.L2.HEIGHT + Units.inchesToMeters(8),
            Rotation2d.fromDegrees(125),
            ControlType.EFFECTOR),
        Optional.of(IntermediateTarget.L2_I),
        Optional.empty()),
    STATION(
        new TargetData(
            0.8468 + Units.inchesToMeters(4),
            Rotation2d.fromRadians(-1.027767),
            ControlType.EFFECTOR),
        Optional.empty(),
        Optional.empty()),
    CLIMB(
        new TargetData(
            ElevatorConstants.ELEVATOR_HEIGHT_PIVOT_MIN + Units.inchesToMeters(0),
            Rotation2d.fromDegrees(0),
            ControlType.PIVOT),
        Optional.empty(),
        Optional.empty()),
    STOW(
        new TargetData(
            ElevatorConstants.ELEVATOR_HEIGHT_PIVOT_SAFETY + Units.inchesToMeters(0),
            Rotation2d.fromDegrees(-90),
            ControlType.PIVOT),
        Optional.empty(),
        Optional.empty()),
    CORAL_STOW(
        new TargetData(
            ElevatorConstants.ELEVATOR_HEIGHT_PIVOT_SAFETY + Units.inchesToMeters(0),
            Rotation2d.fromDegrees(90),
            ControlType.PIVOT),
        Optional.empty(),
        Optional.empty()),
    ALGAE_LOW(
        new TargetData(
            0.9702231159054557 + Units.inchesToMeters(0),
            Rotation2d.fromDegrees(149),
            ControlType.EFFECTOR),
        Optional.empty(),
        Optional.empty()),
    ALGAE_HIGH(
        new TargetData(
            1.2535345791562702 + Units.inchesToMeters(0),
            Rotation2d.fromDegrees(130.79),
            ControlType.EFFECTOR),
        Optional.empty(),
        Optional.empty()),
    ALGAE_PROCESSOR(
        new TargetData(
            0.6381 + Units.inchesToMeters(0), Rotation2d.fromDegrees(-55), ControlType.EFFECTOR),
        Optional.empty(),
        Optional.empty()),
    BARGE(
        new TargetData(
            2.159 + Units.inchesToMeters(0), Rotation2d.fromDegrees(90), ControlType.EFFECTOR),
        Optional.empty(),
        Optional.empty()),
    ALGAE_STOW(
        new TargetData(
            ElevatorConstants.ELEVATOR_HEIGHT_PIVOT_MIN + Units.inchesToMeters(0),
            Rotation2d.fromDegrees(90),
            ControlType.PIVOT),
        Optional.empty(),
        Optional.empty());

    Target(
        TargetData td, Optional<IntermediateTarget> i_En_t, Optional<IntermediateTarget> i_Ex_t) {
      this.td = td;
      this.intermediateEnterTarget = i_En_t;
      this.intermediateExitTarget = i_Ex_t;
    }

    private TargetData td;
    private Optional<IntermediateTarget> intermediateEnterTarget;
    private Optional<IntermediateTarget> intermediateExitTarget;

    public Optional<IntermediateTarget> getIntermediateEnterTarget() {
      return intermediateEnterTarget;
    }

    public Optional<IntermediateTarget> getIntermediateExitTarget() {
      return intermediateExitTarget;
    }

    // Height Methods
    /**
     * Returns the current height including the active offset
     *
     * @return
     */
    public double getHeight() {
      return td.getHeight();
    }

    /**
     * Adjusts the height offset by the supplied increment
     *
     * @param offset
     */
    public void offsetHeight(double offset) {
      td.offsetHeight(offset);
    }

    /**
     * Returns the current stored angle offset
     *
     * @return
     */
    public double getHeightOffset() {
      return td.getHeightOffset();
    }

    /** Reset the height offset to 0 */
    public void resetHeightOffset() {
      td.resetHeightOffset();
    }

    // Angle Methods
    /**
     * Returns the current angle including the active offset
     *
     * @return target angle
     */
    public Rotation2d getAngle() {
      return td.getAngle();
    }

    /**
     * Adjusts the angle offset by the supplied increment
     *
     * @param offset
     */
    public void offsetAngle(Rotation2d offset) {
      td.offsetAngle(offset);
    }

    /**
     * Returns the current stored angle offset
     *
     * @return
     */
    public Rotation2d getAngleOffset() {
      return td.getAngleOffset();
    }

    /** Resets the angle offset to 0 */
    public void resetAngleOffset() {
      td.resetAngleOffset();
    }

    /**
     * Returns the control type associated with the target
     *
     * @return stored control type
     */
    public ControlType getControlType() {
      return td.type_;
    }

    public TargetData getLoggingObject() {
      return td;
    }
  }

  public enum IntermediateTarget {
    L4_I(
        new TargetData(
            FieldConstants.ReefHeight.L4.HEIGHT + Units.inchesToMeters(11),
            Rotation2d.fromDegrees(90),
            ControlType.EFFECTOR)),
    L3_I(
        new TargetData(
            FieldConstants.ReefHeight.L3.HEIGHT + Units.inchesToMeters(8),
            Rotation2d.fromDegrees(90),
            ControlType.EFFECTOR)),
    L2_I(
        new TargetData(
            FieldConstants.ReefHeight.L2.HEIGHT + Units.inchesToMeters(8),
            Rotation2d.fromDegrees(90),
            ControlType.EFFECTOR));

    IntermediateTarget(TargetData td) {
      this.td = td;
    }

    private TargetData td;

    // Height Methods
    /**
     * Returns the current height including the active offset
     *
     * @return
     */
    public double getHeight() {
      return td.getHeight();
    }

    /**
     * Adjusts the height offset by the supplied increment
     *
     * @param offset
     */
    public void offsetHeight(double offset) {
      td.offsetHeight(offset);
    }

    /**
     * Returns the current stored angle offset
     *
     * @return
     */
    public double getHeightOffset() {
      return td.getHeightOffset();
    }

    /** Reset the height offset to 0 */
    public void resetHeightOffset() {
      td.resetHeightOffset();
    }

    // Angle Methods
    /**
     * Returns the current angle including the active offset
     *
     * @return target angle
     */
    public Rotation2d getAngle() {
      return td.getAngle();
    }

    /**
     * Adjusts the angle offset by the supplied increment
     *
     * @param offset
     */
    public void offsetAngle(Rotation2d offset) {
      td.offsetAngle(offset);
    }

    /**
     * Returns the current stored angle offset
     *
     * @return
     */
    public Rotation2d getAngleOffset() {
      return td.getAngleOffset();
    }

    /** Resets the angle offset to 0 */
    public void resetAngleOffset() {
      td.resetAngleOffset();
    }

    /**
     * Returns the control type associated with the target
     *
     * @return stored control type
     */
    public ControlType getControlType() {
      return td.type_;
    }

    public TargetData getLoggingObject() {
      return td;
    }
  }
}
