package frc.mw_lib.proxy_server;

import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.mw_lib.proxy_server.PieceDetectionPacket.PieceDetection;
import frc.mw_lib.proxy_server.TagSolutionPacket.TagSolution;
import frc.mw_lib.util.NumUtil;
import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketAddress;
import java.net.SocketException;
import java.net.SocketTimeoutException;
import java.util.ArrayList;
import java.util.OptionalInt;

public class ProxyServer {

  // Data Packets
  private static OdomPacket odom_packet_ = new OdomPacket();
  private static StatesPacket states_packet_ = new StatesPacket();
  private static TagSolutionPacket tag_solution_packet_ = new TagSolutionPacket();
  private static PieceDetectionPacket piece_detection_packet_ = new PieceDetectionPacket();

  // Socket Config
  private static final int PORT = 5809; // local port to bind server
  private static final String ADDR = "10.41.43.200";
  private static DatagramSocket socket_ = null;
  private static SocketAddress socket_addr_ = null;
  private static final int TIMEOUT = 1; // Server receive blocking timeout

  // Sim stuff
  private static SimDevice sim_cam_;
  private static SimBoolean use_sim_vals_;

  // Coral detection sim
  private static SimDouble sim_tx_;
  private static SimDouble sim_ty_;
  private static SimBoolean sim_is_present_;

  /**
   * Binds the server socket to the set port to begin communication. This must be called once before
   * you can attempt to {@link #updateData()}.
   *
   * @return true: server configured successfully | false: configuration error occurred.
   * @throws SocketException if the socket could not be opened, or the socket could not bind to the
   *     specified local port.
   */
  public static boolean configureServer() {
    sim_cam_ = SimDevice.create("Sim Camera");
    if (sim_cam_ != null) {
      use_sim_vals_ = sim_cam_.createBoolean("sim_detections", Direction.kInput, true);
      sim_is_present_ = sim_cam_.createBoolean("piece present", Direction.kInput, true);
      sim_tx_ = sim_cam_.createDouble("tx", Direction.kInput, 0);
      sim_ty_ = sim_cam_.createDouble("ty", Direction.kInput, 0);
    }

    // check if socket is already bound
    if (socket_ == null || !socket_.isBound()) {
      try {
        socket_ = new DatagramSocket(PORT);
        // set receive blocking timeout (ms)
        socket_.setSoTimeout(TIMEOUT);
        InetAddress addr;
        // try {
        //   addr = InetAddress.getByName(ADDR);
        //   socket_addr_ = new InetSocketAddress(addr, PORT);
        // } catch (UnknownHostException e) {
        //   e.printStackTrace();
        // }
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
        case TagSolutionPacket.TYPE_ID:
          tag_solution_packet_.updateData(buffer);
          break;
        case PieceDetectionPacket.TYPE_ID:
          piece_detection_packet_.updateData(buffer);
          break;
          // Unknown Packet Type
        default:
          return false;
      }

      if (sim_cam_ != null && use_sim_vals_.get()) updateSimDetections();

    } catch (SocketTimeoutException e) {
      if (sim_cam_ != null && use_sim_vals_.get()) updateSimDetections();

      // Timeout occurred
      return false;
    } catch (IOException e) {
      e.printStackTrace();
      return false;
    }

    // packet was processed correctly
    return true;
  }

  private static void updateSimDetections() {
    piece_detection_packet_.piece_detections_.clear();

    if (sim_is_present_.get()) {
      var detection = piece_detection_packet_.new PieceDetection();
      detection.class_id_ = 0;
      detection.theta_x_ = sim_tx_.get();
      detection.theta_y_ = sim_ty_.get();
      detection.detection_index_ = 0;
      detection.detection_count_ = 1;
      piece_detection_packet_.piece_detections_.add(detection);
    }
  }

  /**
   * Gets the current robot Pose. Pose is updated by calling {@link #updateData()} periodically.
   *
   * @return most recent {@link Pose2d} from chassis proxy.
   */
  public static Pose2d getLatestPose() {
    return odom_packet_.pose_;
  }

  /**
   * Gets the current robot Twist. Twist is updated by calling {@link #updateData()} periodically.
   *
   * @return most recent {@link Twist2d} from chassis proxy.
   */
  public static Twist2d getLatestTwist() {
    return odom_packet_.twist_;
  }

  /**
   * Gets the current robot module state. States are updated by calling {@link #updateData()}
   * periodically.
   *
   * @return array containing the {@link SwerveModuleState} for each module
   */
  public static SwerveModuleState[] getLatestModuleStates() {
    return states_packet_.module_states_;
  }

  /**
   * Gets the current tag solution info. Solutions are updated by calling {@link #updateData()}
   *
   * @return {@link TagSolution} with latest Pose and ids used to determine pose.
   */
  public static TagSolution getLatestTagSolution() {
    return tag_solution_packet_.tag_solution_;
  }

  /**
   * Gets the current array list of detected game pieces. Detections are updated by calling {@link
   * #updateData()}
   *
   * @return {@link PieceDetection} ArrayList with latest piece detection data
   */
  public static ArrayList<PieceDetection> getLatestPieceDetections() {
    return piece_detection_packet_.piece_detections_;
  }

  /**
   * Sends snapshot trigger packet for log location flagging
   *
   * @param tag_name flag name to record in log
   */
  public static void snapshot(String tag_name) {
    int tag_name_length = (int) NumUtil.clamp(tag_name.length(), 400);
    byte[] buffer = new byte[1 + tag_name_length];
    buffer[0] = 52; // Message ID
    for (int i = 0; i < tag_name_length; i++) {
      buffer[i + 1] = (byte) Character.getNumericValue(tag_name.charAt(i));
    }

    // try {
    //   socket_.send(new DatagramPacket(buffer, buffer.length, socket_addr_));
    // } catch (IOException e) {
    //   e.printStackTrace();
    // }
  }

  /** Sends match data packet for log name syncing */
  public static void syncMatchData() {
    String event_name = DriverStation.getEventName();
    byte[] buffer = new byte[5 + event_name.length()];
    buffer[0] = 50; // Message ID
    buffer[1] = (byte) DriverStation.getMatchNumber();
    buffer[2] = serializeMatchType();
    buffer[3] = serializeAllianceStation();
    buffer[4] = (byte) event_name.length();
    for (int i = 0; i < event_name.length(); i++) {
      buffer[i + 5] = (byte) Character.getNumericValue(event_name.charAt(i));
    }

    // try {
    //   socket_.send(new DatagramPacket(buffer, buffer.length, socket_addr_));
    // } catch (IOException e) {
    //   e.printStackTrace();
    // }
  }

  /**
   * Serializes {@link DriverStation.MatchType} to byte value {None, Practice, Qualification,
   * Elimination}
   *
   * @return byte value representing match type
   */
  private static byte serializeMatchType() {
    switch (DriverStation.getMatchType()) {
      case Practice:
        {
          return 1;
        }
      case Qualification:
        {
          return 2;
        }
      case Elimination:
        {
          return 3;
        }
      case None:
      default:
        return 0;
    }
  }

  /**
   * Serializes DriverStation Location to byte value {Blue1, Blue2, Blue3, Red1, Red2, Red3} Will
   * return 0 if no DriverStation is Present
   *
   * @return byte value representing station location
   */
  private static byte serializeAllianceStation() {
    OptionalInt optional = DriverStation.getLocation();
    if (optional.isPresent()) {
      int station = optional.getAsInt();
      if (DriverStation.getAlliance().get() == Alliance.Blue) {
        // If on Blue Alliance apply no offset {1, 2, 3}
        return (byte) station;
      } else {
        // If on Red Alliance offset by 3 {4, 5, 6}
        return (byte) (station + 3);
      }
    } else {
      // Drivers Station Not Connected.
      // This should not occur since this will only be TeleOp Init
      return 0;
    }
  }
}
