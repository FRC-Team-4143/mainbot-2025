package frc.lib;
import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.SocketException;
import java.net.SocketTimeoutException;
import java.nio.ByteBuffer;
import java.nio.FloatBuffer;
import java.util.Arrays;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class ChassisProxyServer {

    private static Pose2d pose_ = new Pose2d();
    private static Twist2d twist_ = new Twist2d();
    private static float[] variances_ = new float[3];
    private static double odom_timestamp_ = 0;

    private static SwerveModuleState[] module_states_ = new SwerveModuleState[4];
    private static double states_timestamp_ = 0;

    private static DatagramSocket socket_ = null;

    private static final int PORT = 30000; // local port to bind server
    private static final int TIMEOUT = 1; // Server receive block timeout
    private static final int PACKET_HEADER_SIZE = 9; // number of bytes in packet header 
    private static final int ODOM_PACKET_SIZE = 4*9; // number of data bytes in odom packet
    private static final int STATES_PACKET_SIZE = 4*8; // number of data bytes in states packet

    public static boolean configureServer(){
        // check if socket is already bound
        if (socket_ == null || !socket_.isBound()){
            try {
                socket_ = new DatagramSocket(PORT);
                // set recieve blocking timeout (ms)
                socket_.setSoTimeout(TIMEOUT);
            } catch (SocketException e){
                e.printStackTrace();
                return false;
            }
            // socket configured successfully
            return true;
        }
        // socket already configured
        return true;
    }

    public static boolean updateOdom(){

        try {
            // clear the buffer after every message
            byte[] buffer = new byte[45];
            
            // create a packet to recieve the data
            DatagramPacket packet = new DatagramPacket(buffer, buffer.length);
            
            // recieve the data in byte buffer
            socket_.receive(packet);
    
            // update timestamp
            float seconds = ByteBuffer.wrap(Arrays.copyOfRange(buffer, 1, 4)).getFloat();
            float nanoseconds = ByteBuffer.wrap(Arrays.copyOfRange(buffer, 5, 8)).getFloat();
            double timestamp = seconds + (nanoseconds / 1000000000.0);

            // Determine packet type from first byte of buffer
            switch (buffer[0]){
                // Odom Packet Type
                case 30 : // const uint8_t msg_id{ 30u };
                    if (timestamp > odom_timestamp_){
                        odom_timestamp_ = timestamp;
                        readOdomFromBuffer(ByteBuffer.wrap(Arrays.copyOfRange(buffer, PACKET_HEADER_SIZE, PACKET_HEADER_SIZE+ODOM_PACKET_SIZE-1)).asFloatBuffer());
                    }
                    break;
                // States Packet Type
                case 2 : // const uint8_t msg_id{ 2u };
                    if (timestamp > states_timestamp_){
                        states_timestamp_ = timestamp;
                        readStatesFromBuffer(ByteBuffer.wrap(Arrays.copyOfRange(buffer, PACKET_HEADER_SIZE, PACKET_HEADER_SIZE+STATES_PACKET_SIZE-1)).asFloatBuffer());
                    }
                    break;
                // Unknown Packet Type
                default:
                    return false;
            }
    
        } catch (SocketTimeoutException e){
            // Timeout occured
            return false;
        } catch (IOException e){
            e.printStackTrace();
            return false;
        }
        // packet was procesed correctly
        return true;
    }

    // struct ProxyOdomMsg {
    //     // Unique message identifier
    //     const uint8_t msg_id{ 30u };     | 0
    
    //     // Timestamp
    //     int32_t sec{ 0 };                | 1-4
    //     int32_t nanosec{ 0 };            | 4-8
    
    //     // Position data
    //     int32_t x_pos;                   | 9-12
    //     int32_t y_pos;                   | 13-16
    //     int32_t theta_pos;               | 17-20
    
    //     // Position variances for certainty estimation
    //     int32_t x_var;                   | 21-24
    //     int32_t y_var;                   | 25-28
    //     int32_t theta_var;               | 29-32
    
    //     // Velocity data
    //     int32_t x_dot;                   | 33-36
    //     int32_t y_dot;                   | 37-40
    //     int32_t theta_dot;               | 41-44
    // };
    private static void readOdomFromBuffer(FloatBuffer data){
        pose_ = new Pose2d(data.get(4), data.get(5), new Rotation2d(data.get(6)));
        twist_ = new Twist2d(data.get(10), data.get(11), data.get(12));
        for(int i = 0; i < variances_.length; i++){
            variances_[i] = data.get(7+i);
        }
    }

    // struct ProxyModuleStatesMsg {
    //     // Unique message identifier
    //     const uint8_t msg_id{ 2u };      | 0
    
    //     // Timestamp
    //     int32_t sec{ 0 };                | 1-4
    //     int32_t nanosec{ 0 };            | 4-8
    
    //     // Module 1 data
    //     int32_t ang_pos_1;               | 9-12
    //     int32_t lin_vel_1;               | 13-16
    
    //     // Module 2 data
    //     int32_t ang_pos_2;               | 17-20
    //     int32_t lin_vel_2;               | 21-24
    
    //     // Module 3 data
    //     int32_t ang_pos_3;               | 25-28
    //     int32_t lin_vel_3;               | 29-32
    
    //     // Module 4 data
    //     int32_t ang_pos_4;               | 33-36
    //     int32_t lin_vel_4;               | 37-40
    // };
    private static void readStatesFromBuffer(FloatBuffer data){
        for (int i = 0; i < module_states_.length; i++){
            module_states_[i] = new SwerveModuleState(data.get(3 + (i*2)), new Rotation2d(data.get(4 + (i*2))));
        }
    }

    /**
     * Gets the current robot Pose
     * @return most recent pose from chassis proxy
     */
    public static Pose2d getPose(){
        return pose_;
    }

    /**
     * Gets the current robot Twist
     * @return most recent twist from chassis proxy
     */
    public static Twist2d getTwist(){
        return twist_;
    }
    
    /**
     * Gets the current robot module state
     * @return array containing SwereModuleState for each module
     */
    public static SwerveModuleState[] getModuleStates(){
        return module_states_;
    }
}
