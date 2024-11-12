package frc.lib;
import java.io.IOException;
import java.lang.*;
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
    private static float[] variances_ = new float[3];
    private static SwerveModuleState[] states_ = new SwerveModuleState[4];
    private static float timestamp_ = 0;

    private static DatagramSocket socket_ = null;

    public static boolean configureServer(){
        if (socket_ == null || !socket_.isBound()){
            try {
                socket_ = new DatagramSocket(30000);
                // set recieve blocking timeout (ms)
                socket_.setSoTimeout(1);
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

            // Determine packet type
            // call read...FromBuffer based on packet type
            // switch (packet type){
            //     case: Odom
            //         readOdomFromBuffer(buffer);
            //         break;
            //     case: States
            //         readStatesFromBuffer(buffer);
            //         break;
            //     default:
            //         // unknown packet type
            //         return false;
            // }
    
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
    //     const uint8_t msg_id{ 30u };
    
    //     // Timestamp
    //     int32_t sec{ 0 };
    //     int32_t nanosec{ 0 };
    
    //     // Position data
    //     int32_t x_pos;
    //     int32_t y_pos;
    //     int32_t theta_pos;
    
    //     // Position variances for certainty estimation
    //     int32_t x_var;
    //     int32_t y_var;
    //     int32_t theta_var;
    
    //     // Velocity data
    //     int32_t x_dot;
    //     int32_t y_dot;
    //     int32_t theta_dot;
    // };
    private static void readOdomFromBuffer(byte[] buffer){
        // Convert buffer to floats using Float.intBitsToFloat();
        // pose_ = new Pose2d(buffer[4], buffer[5], new Rotation2d(buffer[6]));
        // twist_ = new Twist2d(buffer[10], buffer[11], buffer[12]);
        // for(int i = 0; i < variances_.length; i++){
        //     variances_[i] = buffer[7+i];
        // }
    }

    // struct ProxyModuleStatesMsg {
    //     // Unique message identifier
    //     const uint8_t msg_id{ 2u };
    
    //     // Timestamp
    //     int32_t sec{ 0 };
    //     int32_t nanosec{ 0 };
    
    //     // Module 1 data
    //     int32_t ang_pos_1;
    //     int32_t lin_vel_1;
    
    //     // Module 2 data
    //     int32_t ang_pos_2;
    //     int32_t lin_vel_2;
    
    //     // Module 3 data
    //     int32_t ang_pos_3;
    //     int32_t lin_vel_3;
    
    //     // Module 4 data
    //     int32_t ang_pos_4;
    //     int32_t lin_vel_4;
    // };
    private static void readStatesFromBuffer(byte[] buffer){
        // Convert buffer to floats using Float.intBitsToFloat();
        // for (int i = 0; i < states_.length; i++){
        //     states_[i] = new SwerveModuleState(buffer[3 + (i*2)], buffer[4 + (i*2)]);
        // }
    }

    public static Pose2d getPose(){
        return pose_;
    }

    public static Twist2d getTwist(){
        return twist_;
    }
    
    public static SwerveModuleState[] getStates(){
        return states_;
    }
}
