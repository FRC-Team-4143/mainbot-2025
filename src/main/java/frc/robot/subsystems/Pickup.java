package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import frc.mw_lib.subsystem.Subsystem;
import monologue.Logged;

public class Pickup extends Subsystem {
    TalonFX clawMotor;
    TalonFX clawRotateMotor;

    enum PickupStages {
        ROTATE_IN,
        ROTATE_OUT,
        SCORE,
        PICKUP,
    }
    /**
     * Reads all sensors and stores periodic data
     */
    public void readPeriodicInputs(double timestamp) {
        
    }

    /**
     * Computes updated outputs for the actuators
     */
    public void updateLogic(double timestamp) {
       
    }

    /**
     * Writes the periodic outputs to actuators (motors and etc...)
     */
    public void writePeriodicOutputs(double timestamp) {

    }

    /**
     * Outputs all logging information to the SmartDashboard
     */
    public void outputTelemetry(double timestamp) {

    }

    /**
     * Get logging object from subsystem
     */
    public Logged getLoggingObject() {
        return null;
    }

    /**
     * Called to reset and configure the subsystem
     */
    public void reset() {}
}
