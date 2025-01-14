package frc.robot.subsystems;
import frc.lib.subsystem.Subsystem;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Annotations.Log;
import monologue.Logged;
import com.playingwithfusion.TimeOfFlight;

public class Elevator extends Subsystem {
  // Elevator/Arm motors
  TalonFX elevator_base_motor_;
  TalonFX arm_vertical_movement_motor_;

  // Enums for Elevator/Arm
  public enum Level {
    L1,
    L2,
    L3,
    L4
  }

  // Additional Enum goes here???

  private ElevatorPeriodicIo io_;
  DigitalInput elevator_starting_point_;

  // Constructor
  public Elevator() {
    io_ = new ElevatorPeriodicIo();
    
    // Limit Switch: Elevator 
    // Change channel once we find out what port it goes into on the RoboRIO
    elevator_starting_point_ = new DigitalInput(0);

  }

  /** Reads all sensors and stores periodic data */
  public void readPeriodicInputs(double timestamp) {
    io_.is_elevator_at_starting_point_ = elevator_starting_point_.get();
    // io_.is_coral_piece_loaded_ = 

  }
    

  /** Computes updated outputs for the actuators */
  public void updateLogic(double timestamp){

  }

  /** Writes the periodic outputs to actuators (motors and etc...) */
  public void writePeriodicOutputs(double timestamp){}

  /** Outputs all logging information to the SmartDashboard */
  public void outputTelemetry(double timestamp){}

  /** Get logging object from subsystem */
  public Logged getLoggingObject(){
    return io_;
  }

  public class ElevatorPeriodicIo implements Logged {
    // IO container for all variables
    @Log.File
    public boolean is_elevator_at_starting_point_ = false;
    @Log.File
    public boolean is_coral_piece_loaded_ = false;
  }

  /** Called to reset and configure the subsystem */
  public void reset(){
  }
}
