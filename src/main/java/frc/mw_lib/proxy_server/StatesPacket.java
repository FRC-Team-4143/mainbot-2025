package frc.mw_lib.proxy_server;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import java.nio.ByteBuffer;

public class StatesPacket implements Packet {

  // Byte index for state packet data
  public static final int TYPE_ID = 2;
  private static final int MOD_1_ANG_POS_IDX = 9;
  private static final int MOD_1_LIN_VEL_IDX = 13;
  private static final int MOD_2_ANG_POS_IDX = 17;
  private static final int MOD_2_LIN_VEL_IDX = 21;
  private static final int MOD_3_ANG_POS_IDX = 25;
  private static final int MOD_3_LIN_VEL_IDX = 29;
  private static final int MOD_4_ANG_POS_IDX = 33;
  private static final int MOD_4_LIN_VEL_IDX = 37;

  private static final double RESOLUTION = 1000.0;

  public SwerveModuleState[] module_states_ = new SwerveModuleState[4];
  private Timestamp timestamp_ = new Timestamp(0, 0);

  StatesPacket() {}

  public void updateData(byte[] buffer) {
    Timestamp timestamp =
        new Timestamp(
            ByteBuffer.wrap(buffer, TIME_SEC_IDX, 4).getInt(),
            ByteBuffer.wrap(buffer, TIME_NSEC_IDX, 4).getInt());
    if (timestamp.isLatest(timestamp_)) {
      timestamp_ = timestamp;
      module_states_[0] =
          new SwerveModuleState(
              ByteBuffer.wrap(buffer, MOD_1_LIN_VEL_IDX, 4).getInt() / RESOLUTION,
              new Rotation2d(ByteBuffer.wrap(buffer, MOD_1_ANG_POS_IDX, 4).getInt() / RESOLUTION));
      module_states_[1] =
          new SwerveModuleState(
              ByteBuffer.wrap(buffer, MOD_2_LIN_VEL_IDX, 4).getInt() / RESOLUTION,
              new Rotation2d(ByteBuffer.wrap(buffer, MOD_2_ANG_POS_IDX, 4).getInt() / RESOLUTION));
      module_states_[2] =
          new SwerveModuleState(
              ByteBuffer.wrap(buffer, MOD_3_LIN_VEL_IDX, 4).getInt() / RESOLUTION,
              new Rotation2d(ByteBuffer.wrap(buffer, MOD_3_ANG_POS_IDX, 4).getInt() / RESOLUTION));
      module_states_[3] =
          new SwerveModuleState(
              ByteBuffer.wrap(buffer, MOD_4_LIN_VEL_IDX, 4).getInt() / RESOLUTION,
              new Rotation2d(ByteBuffer.wrap(buffer, MOD_4_ANG_POS_IDX, 4).getInt() / RESOLUTION));
    }
  }
}
