package frc.lib;
import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.SocketException;
import java.net.SocketTimeoutException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class ChassisProxyServer {

    private static Pose2d pose_ = new Pose2d();
    private static Twist2d twist_ = new Twist2d();
    private static double[] variances_ = new double[3];
    private static Timestamp odom_timestamp_ = new Timestamp(0, 0);

    private static SwerveModuleState[] module_states_ = new SwerveModuleState[4];
    private static Timestamp states_timestamp_ = new Timestamp(0, 0);

    private static DatagramSocket socket_ = null;

    private static final int PORT = 30000; // local port to bind server
    private static final int TIMEOUT = 1; // Server receive block timeout

    public static class Timestamp {
        public int seconds;
        public int nanoseconds;

        public Timestamp(int seconds, int nanoseconds){
            this.seconds = seconds;
            this.nanoseconds = nanoseconds;
        }

        public boolean isLater(Timestamp compare){
            if(seconds > compare.seconds){
                return false;
            } else if (nanoseconds > compare.nanoseconds) {
                return false;
            } else {
                return true;
            }
        }

        public double toDouble(){
            return seconds + nanoseconds / 1000000000.0;
        }
    }

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

    public static boolean updateData(){

        try {
            // clear the buffer after every message
            byte[] buffer = new byte[45];
            
            // create a packet to recieve the data
            DatagramPacket packet = new DatagramPacket(buffer, buffer.length);
            
            // recieve the data in byte buffer
            socket_.receive(packet);

    
            // update timestamp
            Timestamp timestamp = new Timestamp(getIntFromBuffer(buffer, 1), getIntFromBuffer(buffer, 5));
            System.out.println("ID: " + ((int) buffer[0]) + " | Seconds: " + timestamp.toDouble());

            // Determine packet type from first byte of buffer
            switch ((int) buffer[0]){
                // Odom Packet Type
                case 30 : // const uint8_t msg_id{ 30u };
                    if (timestamp.isLater(odom_timestamp_)){
                        odom_timestamp_ = timestamp;
                        readOdomFromBuffer(buffer);
                    }
                    break;
                // States Packet Type
                case 2 : // const uint8_t msg_id{ 2u };
                    if (timestamp.isLater(states_timestamp_)){
                        states_timestamp_ = timestamp;
                        readStatesFromBuffer(buffer);
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
    private static void readOdomFromBuffer(byte[] buffer){
        pose_ = new Pose2d(getIntFromBuffer(buffer, 9)/1000.0, getIntFromBuffer(buffer, 13)/1000.0, new Rotation2d(getIntFromBuffer(buffer, 17)/1000.0));
        twist_ = new Twist2d(getIntFromBuffer(buffer, 33)/1000.0, getIntFromBuffer(buffer, 37)/1000.0, getIntFromBuffer(buffer, 41)/1000.0);
            
        variances_[0] = getIntFromBuffer(buffer, 21)/1000.0;
        variances_[1] = getIntFromBuffer(buffer, 25)/1000.0;
        variances_[2] = getIntFromBuffer(buffer, 29)/1000.0;
    }

    // struct ProxyModuleStatesMsg {
    //     // Unique message identifier
    //     const uint8_t msg_id{ 2u };      | 0
    
    //     // Timestamp
    //     int32_t sec{ 0 };                | 1-4
    //     int32_t nanosec{ 0 };            | 4-8
    
    //     // Module 1 buffer
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
    private static void readStatesFromBuffer(byte[] buffer){
        for (int i = 0; i < module_states_.length; i++){
            module_states_[i] = new SwerveModuleState(getIntFromBuffer(buffer, 9+i*8)/1000.0, new Rotation2d(getIntFromBuffer(buffer, 13+i*8)/1000.0));
        }
    }

    private static int getIntFromBuffer(byte[] buffer, int start){
        return (((int) buffer[start]) << 24) | (((int) buffer[start+1]) << 16) | (((int) buffer[start+2]) << 8) | ((int) buffer[start+3]);
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
