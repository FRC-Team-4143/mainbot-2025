package frc.mw_lib.proxy_server;

import java.nio.ByteBuffer;
import java.util.ArrayList;

public class PieceDetectionPacket implements Packet {

  public ArrayList<PieceDetection> piece_detections_ = new ArrayList<>();

  public class PieceDetection {
    public int class_id_ = 0;
    public int detection_count_ = 0;
    public int detection_index_ = 0;
    public double theta_x_ = 0.0; // Fixed precision 64 bit int
    public double theta_y_ = 0.0; // Fixed precision 64 bit int

    public PieceDetection() {}
  }

  // Byte index for detection solution packet data
  public static final int TYPE_ID = 10;
  private static final int DETECTION_COUNT = 9;
  private static final int DETECTION_INDEX = 13;
  private static final int CLASS_ID = 17;
  private static final int THETA_X = 18;
  private static final int THETA_Y = 26;

  private static final double RESOLUTION = 1e6;

  private Timestamp timestamp_ = new Timestamp(0, 0);

  public void updateData(byte[] buffer) {

    Timestamp timestamp =
        new Timestamp(
            ByteBuffer.wrap(buffer, TIME_SEC_IDX, 4).getInt(),
            ByteBuffer.wrap(buffer, TIME_NSEC_IDX, 4).getInt());
    if (timestamp.isLatest(timestamp_)) {
      timestamp_ = timestamp;
      PieceDetection piece_detection = new PieceDetection();
      piece_detection.detection_count_ = ByteBuffer.wrap(buffer, DETECTION_COUNT, 4).getInt();
      piece_detection.detection_index_ = ByteBuffer.wrap(buffer, DETECTION_INDEX, 4).getInt();
      if (piece_detection.detection_count_ == 0 || piece_detection.detection_index_ == 0) {
        piece_detections_.clear();
      }

      if (piece_detection.detection_count_ != 0) {
        piece_detection.class_id_ = ByteBuffer.wrap(buffer, CLASS_ID, 1).get();
        piece_detection.theta_x_ = ByteBuffer.wrap(buffer, THETA_X, 8).getLong() / RESOLUTION;
        piece_detection.theta_y_ = ByteBuffer.wrap(buffer, THETA_Y, 8).getLong() / RESOLUTION;
        piece_detections_.add(piece_detection.detection_index_, piece_detection);
      }
    }
  }
}
