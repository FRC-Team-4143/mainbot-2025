package frc.lib;

import edu.wpi.first.math.geometry.Translation2d;

public class ElevatorTargets {
    public enum TargetType {
        SAFETY(
                new TargetData(new Translation2d(), "SAFETY"),
                new Translation2d[0],
                new Translation2d[0]),
        L4(
                new TargetData(new Translation2d(), "L4"),
                new Translation2d[0],
                new Translation2d[0]),
        L3(
                new TargetData(new Translation2d(), "L3"),
                new Translation2d[0],
                new Translation2d[0]),
        L2(
                new TargetData(new Translation2d(), "L2"),
                new Translation2d[0],
                new Translation2d[0]),
        L1(
                new TargetData(new Translation2d(), "L1"),
                new Translation2d[0],
                new Translation2d[0]),
        STATION(
                new TargetData(new Translation2d(), "STATION"),
                new Translation2d[0],
                new Translation2d[0]),
        CLIMB(
                new TargetData(new Translation2d(), "CLIMB"),
                new Translation2d[0],
                new Translation2d[0]),
        CORAL_INTAKE(
                new TargetData(new Translation2d(), "CORAL_INTAKE"),
                new Translation2d[0],
                new Translation2d[0]),
        CORAL_STOW(
                new TargetData(new Translation2d(), "CORAL_STOW"),
                new Translation2d[0],
                new Translation2d[0]),
        ALGAE_LOW(
                new TargetData(new Translation2d(), "ALGAE_LOW"),
                new Translation2d[0],
                new Translation2d[0]),
        ALGAE_HIGH(
                new TargetData(new Translation2d(), "ALGAE_HIGH"),
                new Translation2d[0],
                new Translation2d[0]),
        ALGAE_PROCESSOR(
                new TargetData(new Translation2d(), "ALGAE_PROCESSOR"),
                new Translation2d[0],
                new Translation2d[0]),
        BARGE(
                new TargetData(new Translation2d(), "BARGE"),
                new Translation2d[0],
                new Translation2d[0]),
        ALGAE_STOW(
                new TargetData(new Translation2d(), "ALGAE_STOW"),
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

        public Translation2d[] getEnterTrj() {
            return enter_trj;
        }

        public Translation2d[] getExitTrj() {
            return exit_trj;
        }

        public TargetData getTarget() {
            return target;
        }

        public void offsetY(double offset) {
            target.offsetY(offset);
        }

        public void resetYOffset() {
            target.resetYOffset();
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
