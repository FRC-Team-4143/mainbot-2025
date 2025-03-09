package frc.mw_lib.proxy_server;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.SocketException;
import java.net.SocketTimeoutException;

public class ChassisProxyServer {

  // Data Packets
  private static OdomPacket odom_packet_ = new OdomPacket();
  private static StatesPacket states_packet_ = new StatesPacket();
  private static DetectionPacket detection_packet_ = new DetectionPacket();

  // Socket Config
  private static DatagramSocket socket_ = null;
  private static final int PORT = 1180; // local port to bind server
  private static final int TIMEOUT = 1; // Server receive blocking timeout

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

      // Determine packet type from first byte of buffer
      switch ((int) buffer[Packet.ID_IDX]) {
          // Odom Packet Type
        case OdomPacket.TYPE_ID: // const uint8_t msg_id{ 30u };
          odom_packet_.updateData(buffer);
          break;
          // States Packet Type
        case StatesPacket.TYPE_ID: // const uint8_t msg_id{ 2u };
          states_packet_.updateData(buffer);
          break;
        case DetectionPacket.TYPE_ID:
          detection_packet_.updateData(buffer);
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
   * Gets the current robot Pose. Pose is updated by calling {@link #updateData()} periodically.
   *
   * @return most recent {@link Pose2d} from chassis proxy.
   */
  public static Pose2d getPose() {
    return odom_packet_.pose_;
  }

  /**
   * Gets the current robot Twist. Twist is updated by calling {@link #updateData()} periodically.
   *
   * @return most recent {@link Twist2d} from chassis proxy.
   */
  public static Twist2d getTwist() {
    return odom_packet_.twist_;
  }

  /**
   * Gets the current robot module state. States are updated by calling {@link #updateData()}
   * periodically.
   *
   * @return array containing the {@link SwerveModuleState} for each module
   */
  public static SwerveModuleState[] getModuleStates() {
    return states_packet_.module_states_;
  }
}
