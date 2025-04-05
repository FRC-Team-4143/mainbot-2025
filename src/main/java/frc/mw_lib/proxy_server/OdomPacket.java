package frc.mw_lib.proxy_server;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import java.nio.ByteBuffer;

public class OdomPacket implements Packet {

  // Byte index for odom packet data
  public static final int TYPE_ID = 30;
  private static final int X_POS_IDX = 9;
  private static final int Y_POS_IDX = 13;
  private static final int OMEGA_POS_IDX = 17;
  private static final int X_VAR_IDX = 21;
  private static final int Y_VAR_IDX = 25;
  private static final int OMEGA_VAR_IDX = 29;
  private static final int X_DOT_IDX = 33;
  private static final int Y_DOT_IDX = 37;
  private static final int OMEGA_DOT_IDX = 41;

  private static final double RESOLUTION = 1000.0;

  public Pose2d pose_ = new Pose2d();
  public Twist2d twist_ = new Twist2d();
  public double[] variances_ = new double[3];
  private Timestamp timestamp_ = new Timestamp(0, 0);

  OdomPacket() {}

  public void updateData(byte[] buffer) {
    Timestamp timestamp =
        new Timestamp(
            ByteBuffer.wrap(buffer, TIME_SEC_IDX, 4).getInt(),
            ByteBuffer.wrap(buffer, TIME_NSEC_IDX, 4).getInt());
    if (timestamp.isLatest(timestamp_)) {
      timestamp_ = timestamp;
      // Position data
      pose_ =
          new Pose2d(
              ByteBuffer.wrap(buffer, X_POS_IDX, 4).getInt() / RESOLUTION,
              ByteBuffer.wrap(buffer, Y_POS_IDX, 4).getInt() / RESOLUTION,
              new Rotation2d(ByteBuffer.wrap(buffer, OMEGA_POS_IDX, 4).getInt() / RESOLUTION));

      // Velocity data
      twist_ =
          new Twist2d(
              ByteBuffer.wrap(buffer, X_DOT_IDX, 4).getInt() / RESOLUTION,
              ByteBuffer.wrap(buffer, Y_DOT_IDX, 4).getInt() / RESOLUTION,
              ByteBuffer.wrap(buffer, OMEGA_DOT_IDX, 4).getInt() / RESOLUTION);

      // Position variances for certainty estimation
      variances_[0] = ByteBuffer.wrap(buffer, X_VAR_IDX, 4).getInt() / RESOLUTION;
      variances_[1] = ByteBuffer.wrap(buffer, Y_VAR_IDX, 4).getInt() / RESOLUTION;
      variances_[2] = ByteBuffer.wrap(buffer, OMEGA_VAR_IDX, 4).getInt() / RESOLUTION;
    }
  }
}
