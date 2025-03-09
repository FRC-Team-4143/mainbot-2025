package frc.mw_lib.proxy_server;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.nio.ByteBuffer;

public class DetectionPacket implements Packet {

  // Byte index for detection solution packet data
  public static final int TYPE_ID = 15;
  private static final int X_POS_IDX = 9;
  private static final int Y_POS_IDX = 13;
  private static final int OMEGA_POS_IDX = 17;
  private static final int DETECTED_TAG_SIZE = 21;
  private static final int DETECTED_TAG_START = 25;

  public Pose2d pose_ = new Pose2d();
  public int[] detected_ids_ = new int[22];
  private Timestamp timestamp_ = new Timestamp(0, 0);

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
              ByteBuffer.wrap(buffer, X_POS_IDX, 4).getInt() / 1000.0,
              ByteBuffer.wrap(buffer, Y_POS_IDX, 4).getInt() / 1000.0,
              new Rotation2d(ByteBuffer.wrap(buffer, OMEGA_POS_IDX, 4).getInt() / 1000.0));

      int detected_tag_size = ByteBuffer.wrap(buffer, DETECTED_TAG_SIZE, 4).getInt();
      detected_ids_ = new int[detected_tag_size];
      for (int i = 0; i < detected_tag_size; i++) {
        detected_ids_[i] = ByteBuffer.wrap(buffer, DETECTED_TAG_START + (i * 4), 4).getInt();
      }
    }
  }
}
