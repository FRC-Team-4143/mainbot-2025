package frc.mw_lib.proxy_server;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.nio.ByteBuffer;
import java.util.ArrayList;

public class DetectionPacket implements Packet {

  public class Detection {
    public Pose2d pose_ = new Pose2d();
    public ArrayList<Integer> detected_ids_ = new ArrayList<>();
  }

  // Byte index for detection solution packet data
  public static final int TYPE_ID = 15;
  private static final int X_POS_IDX = 9;
  private static final int Y_POS_IDX = 13;
  private static final int OMEGA_POS_IDX = 17;
  private static final int DETECTED_TAG_SIZE = 21;
  private static final int DETECTED_TAG_START = 25;

  public Detection detection_ = new Detection();
  private Timestamp timestamp_ = new Timestamp(0, 0);

  public void updateData(byte[] buffer) {
    Timestamp timestamp =
        new Timestamp(
            ByteBuffer.wrap(buffer, TIME_SEC_IDX, 4).getInt(),
            ByteBuffer.wrap(buffer, TIME_NSEC_IDX, 4).getInt());
    if (timestamp.isLatest(timestamp_)) {
      timestamp_ = timestamp;
      // Position data
      detection_.pose_ =
          new Pose2d(
              ByteBuffer.wrap(buffer, X_POS_IDX, 4).getInt() / 1000.0,
              ByteBuffer.wrap(buffer, Y_POS_IDX, 4).getInt() / 1000.0,
              new Rotation2d(ByteBuffer.wrap(buffer, OMEGA_POS_IDX, 4).getInt() / 1000.0));

      // Detection ID data
      detection_.detected_ids_.clear();
      int detected_tag_size = ByteBuffer.wrap(buffer, DETECTED_TAG_SIZE, 4).getInt();
      for (int i = 0; i < detected_tag_size; i++) {
        detection_.detected_ids_.add(ByteBuffer.wrap(buffer, DETECTED_TAG_START + (i * 4), 4).getInt());
      }
    }
  }
}
