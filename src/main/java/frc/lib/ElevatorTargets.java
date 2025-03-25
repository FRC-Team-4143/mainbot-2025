package frc.lib;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.lib.TargetData.ControlType;
import frc.robot.Constants.ElevatorConstants;
import java.util.Optional;

public class ElevatorTargets {
  private static final TargetData L4_INT =
      new TargetData(
          FieldConstants.ReefHeight.L4.HEIGHT + Units.inchesToMeters(11),
          Rotation2d.fromDegrees(90),
          ControlType.PIVOT);

  private static final TargetData L3_INT =
      new TargetData(
          FieldConstants.ReefHeight.L3.HEIGHT + Units.inchesToMeters(8),
          Rotation2d.fromDegrees(90),
          ControlType.PIVOT);

  private static final TargetData L2_INT =
      new TargetData(
          ElevatorConstants.ELEVATOR_HEIGHT_PIVOT_SAFETY + Units.inchesToMeters(12),
          Rotation2d.fromDegrees(90),
          ControlType.PIVOT);

  private static final TargetData L1_INT =
      new TargetData(
          FieldConstants.ReefHeight.L1.HEIGHT + Units.inchesToMeters(8),
          Rotation2d.fromDegrees(90),
          ControlType.PIVOT);

  private static final TargetData STOW_ENTER =
      new TargetData(
          ElevatorConstants.ELEVATOR_HEIGHT_PIVOT_SAFETY + Units.inchesToMeters(12),
          Rotation2d.fromDegrees(-120),
          ControlType.PIVOT);

  private static final TargetData STOW_EXIT =
      new TargetData(
          ElevatorConstants.ELEVATOR_HEIGHT_PIVOT_SAFETY + Units.inchesToMeters(12),
          Rotation2d.fromDegrees(-120),
          ControlType.PIVOT);

  private static final TargetData ALGAE_STOW_ENTER =
      new TargetData(
          ElevatorConstants.ELEVATOR_HEIGHT_PIVOT_SAFETY + Units.inchesToMeters(12),
          Rotation2d.fromDegrees(90),
          ControlType.PIVOT);

  public enum TargetType {
    SAFETY(
        new TargetData(
            ElevatorConstants
                .ELEVATOR_HEIGHT_PIVOT_SAFETY, // height is ignored for this special target
            Rotation2d.fromDegrees(90),
            ControlType.PIVOT),
        Optional.empty(),
        Optional.empty()),
    L4(
        new TargetData(
            FieldConstants.ReefHeight.L4.HEIGHT + Units.inchesToMeters(11),
            Rotation2d.fromDegrees(145.3),
            ControlType.EFFECTOR),
        Optional.of(L4_INT),
        Optional.empty()),
    L3(
        new TargetData(
            FieldConstants.ReefHeight.L3.HEIGHT + Units.inchesToMeters(8),
            Rotation2d.fromDegrees(125),
            ControlType.EFFECTOR),
        Optional.of(L3_INT),
        Optional.empty()),
    L2(
        new TargetData(
            FieldConstants.ReefHeight.L2.HEIGHT + Units.inchesToMeters(8),
            Rotation2d.fromDegrees(125),
            ControlType.EFFECTOR),
        Optional.of(L2_INT),
        Optional.empty()),
    L1(
        new TargetData(
            FieldConstants.ReefHeight.L2.HEIGHT + Units.inchesToMeters(8),
            Rotation2d.fromDegrees(125),
            ControlType.EFFECTOR),
        Optional.of(L1_INT),
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
    CORAL_INTAKE(
        new TargetData(
            ElevatorConstants.ELEVATOR_HEIGHT_PIVOT_SAFETY + Units.inchesToMeters(0),
            Rotation2d.fromDegrees(-120),
            ControlType.PIVOT),
        Optional.of(STOW_ENTER),
        Optional.of(STOW_EXIT)),
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
        Optional.of(ALGAE_STOW_ENTER),
        Optional.empty());

    TargetType(TargetData target, Optional<TargetData> enter_tgt, Optional<TargetData> exit_tgt) {
      this.target = target;
      this.int_enter_tgt = enter_tgt;
      this.int_exit_tgt = exit_tgt;
    }

    private TargetData target;
    private Optional<TargetData> int_enter_tgt;
    private Optional<TargetData> int_exit_tgt;

    public Optional<TargetData> getEnterTarget() {
      return int_enter_tgt;
    }

    public Optional<TargetData> getExitTarget() {
      return int_exit_tgt;
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
    public void offsetAngle(Rotation2d offset) {
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
