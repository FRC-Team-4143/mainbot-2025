package frc.mw_lib.proxy_server;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.SocketException;
import java.net.SocketTimeoutException;
import java.nio.ByteBuffer;

public class ChassisProxyServer {

  // Byte index for packet header data
  private static final int ID_IDX = 0;
  private static final int TIME_SEC_IDX = 1;
  private static final int TIME_NSEC_IDX = 5;

  // Byte index for odom packet data
  private static final int X_POS_IDX = 9;
  private static final int Y_POS_IDX = 13;
  private static final int OMEGA_POS_IDX = 17;
  private static final int X_VAR_IDX = 21;
  private static final int Y_VAR_IDX = 25;
  private static final int OMEGA_VAR_IDX = 29;
  private static final int X_DOT_IDX = 33;
  private static final int Y_DOT_IDX = 37;
  private static final int OMEGA_DOT_IDX = 41;

  // Byte index for state packet data
  private static final int MOD_1_ANG_POS_IDX = 9;
  private static final int MOD_1_LIN_VEL_IDX = 13;
  private static final int MOD_2_ANG_POS_IDX = 17;
  private static final int MOD_2_LIN_VEL_IDX = 21;
  private static final int MOD_3_ANG_POS_IDX = 25;
  private static final int MOD_3_LIN_VEL_IDX = 29;
  private static final int MOD_4_ANG_POS_IDX = 33;
  private static final int MOD_4_LIN_VEL_IDX = 37;

  // Data members
  private static Pose2d pose_ = new Pose2d();
  private static Twist2d twist_ = new Twist2d();
  private static double[] variances_ = new double[3];
  private static Timestamp odom_timestamp_ = new Timestamp(0, 0);

  private static SwerveModuleState[] module_states_ = new SwerveModuleState[4];
  private static Timestamp states_timestamp_ = new Timestamp(0, 0);

  // Socket Config
  private static DatagramSocket socket_ = null;
  private static final int PORT = 1180; // local port to bind server
  private static final int TIMEOUT = 1; // Server receive blocking timeout

  // Timestamp class to store packet timestamp and preform operations
  public static class Timestamp {
    public int seconds;
    public int nanoseconds;

    public Timestamp(int seconds, int nanoseconds) {
      this.seconds = seconds;
      this.nanoseconds = nanoseconds;
    }

    /**
     * Determines is current timestamp is more recent than supplied timestamp
     *
     * @param recorded the store {@link #Timestamp} to compare current timestamp against
     * @return true: current timestamp is the most recent | false: recorded timestamp is the most
     *     recent
     */
    public boolean isLatest(Timestamp recorded) {
      if (seconds > recorded.seconds) {
        return true;
      } else if (nanoseconds > recorded.nanoseconds) {
        return true;
      } else {
        return false;
      }
    }
  }

  /**
   * Binds the server socket to the set port to begin communication. This must be called once before
   * you can attempt to {@link #updateData()}.
   *
   * @return true: server configured successfully | false: configuration error occurred.
   * @throws SocketException if the socket could not be opened, or the socket could not bind to the
   *     specified local port.
   */
  public static boolean configureServer() {
    // check if socket is already bound
    if (socket_ == null || !socket_.isBound()) {
      try {
        socket_ = new DatagramSocket(PORT);
        // set receive blocking timeout (ms)
        socket_.setSoTimeout(TIMEOUT);
      } catch (SocketException e) {
        e.printStackTrace();
        return false;
      }
      // socket configured successfully
      return true;
    }
    // socket already configured
    return true;
  }

  /**
   * Server attempts to receive data from bound socket and parse the incoming packet. This method is
   * called periodically to continuously update internal members. Ensure {@link #configureServer()}
   *
   * @return true: packet successfully received | false: receive error occurred.
   * @throws SocketTimeoutException if socket receive timed out to avoid blocking.
   * @throws IOException if I/O error occurs.
   */
  public static boolean updateData() {

    try {
      // clear the buffer after every message
      byte[] buffer = new byte[45];

      // create a packet to receive the data
      DatagramPacket packet = new DatagramPacket(buffer, buffer.length);

      // receive the data in byte buffer
      socket_.receive(packet);

      // update timestamp
      Timestamp timestamp =
          new Timestamp(
              ByteBuffer.wrap(buffer, TIME_SEC_IDX, 4).getInt(),
              ByteBuffer.wrap(buffer, TIME_NSEC_IDX, 4).getInt());

      // Determine packet type from first byte of buffer
      switch ((int) buffer[ID_IDX]) {
          // Odom Packet Type
        case 30: // const uint8_t msg_id{ 30u };
          if (timestamp.isLatest(odom_timestamp_)) {
            odom_timestamp_ = timestamp;
            readOdomFromBuffer(buffer);
          }
          break;
          // States Packet Type
        case 2: // const uint8_t msg_id{ 2u };
          if (timestamp.isLatest(states_timestamp_)) {
            states_timestamp_ = timestamp;
            readStatesFromBuffer(buffer);
          }
          break;
          // Unknown Packet Type
        default:
          return false;
      }

    } catch (SocketTimeoutException e) {
      // Timeout occurred
      return false;
    } catch (IOException e) {
      e.printStackTrace();
      return false;
    }
    // packet was processed correctly
    return true;
  }

  /**
   * Updates internal pose, twist, and variance with data from byte buffer.
   *
   * @param buffer byte buffer containing data from packet to be parsed.
   */
  private static void readOdomFromBuffer(byte[] buffer) {
    // Position data
    pose_ =
        new Pose2d(
            ByteBuffer.wrap(buffer, X_POS_IDX, 4).getInt() / 1000.0,
            ByteBuffer.wrap(buffer, Y_POS_IDX, 4).getInt() / 1000.0,
            new Rotation2d(ByteBuffer.wrap(buffer, OMEGA_POS_IDX, 4).getInt() / 1000.0));

    // Velocity data
    twist_ =
        new Twist2d(
            ByteBuffer.wrap(buffer, X_DOT_IDX, 4).getInt() / 1000.0,
            ByteBuffer.wrap(buffer, Y_DOT_IDX, 4).getInt() / 1000.0,
            ByteBuffer.wrap(buffer, OMEGA_DOT_IDX, 4).getInt() / 1000.0);

    // Position variances for certainty estimation
    variances_[0] = ByteBuffer.wrap(buffer, X_VAR_IDX, 4).getInt() / 1000.0;
    variances_[1] = ByteBuffer.wrap(buffer, Y_VAR_IDX, 4).getInt() / 1000.0;
    variances_[2] = ByteBuffer.wrap(buffer, OMEGA_VAR_IDX, 4).getInt() / 1000.0;
  }

  /**
   * Updates internal module states with data from byte buffer.
   *
   * @param buffer byte buffer containing data from packet to be parsed.
   */
  private static void readStatesFromBuffer(byte[] buffer) {
    module_states_[0] =
        new SwerveModuleState(
            ByteBuffer.wrap(buffer, MOD_1_LIN_VEL_IDX, 4).getInt() / 1000.0,
            new Rotation2d(ByteBuffer.wrap(buffer, MOD_1_ANG_POS_IDX, 4).getInt() / 1000.0));
    module_states_[1] =
        new SwerveModuleState(
            ByteBuffer.wrap(buffer, MOD_2_LIN_VEL_IDX, 4).getInt() / 1000.0,
            new Rotation2d(ByteBuffer.wrap(buffer, MOD_2_ANG_POS_IDX, 4).getInt() / 1000.0));
    module_states_[2] =
        new SwerveModuleState(
            ByteBuffer.wrap(buffer, MOD_3_LIN_VEL_IDX, 4).getInt() / 1000.0,
            new Rotation2d(ByteBuffer.wrap(buffer, MOD_3_ANG_POS_IDX, 4).getInt() / 1000.0));
    module_states_[3] =
        new SwerveModuleState(
            ByteBuffer.wrap(buffer, MOD_4_LIN_VEL_IDX, 4).getInt() / 1000.0,
            new Rotation2d(ByteBuffer.wrap(buffer, MOD_4_ANG_POS_IDX, 4).getInt() / 1000.0));
  }

  /**
   * Gets the current robot Pose. Pose is updated by calling {@link #updateData()} periodically.
   *
   * @return most recent {@link Pose2d} from chassis proxy.
   */
  public static Pose2d getPose() {
    return pose_;
  }

  /**
   * Gets the current robot Twist. Twist is updated by calling {@link #updateData()} periodically.
   *
   * @return most recent {@link Twist2d} from chassis proxy.
   */
  public static Twist2d getTwist() {
    return twist_;
  }

  /**
   * Gets the current robot module state. States are updated by calling {@link #updateData()}
   * periodically.
   *
   * @return array containing the {@link SwerveModuleState} for each module
   */
  public static SwerveModuleState[] getModuleStates() {
    return module_states_;
  }
}
