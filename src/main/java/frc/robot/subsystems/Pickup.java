package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.mw_lib.subsystem.Subsystem;
import frc.robot.Constants;
import frc.robot.commands.PickupLoad;
import monologue.Annotations.Log;
import monologue.Logged;

public class Pickup extends Subsystem {
    private PickupPeriodicIo io_;
    private TalonFX rotate_motor_;
    private TalonFX roller_motor_;
    private TimeOfFlight algae_loaded_sensor_;
    private DigitalInput rotate_idle_switch_;
    private DigitalInput rotate_pickup_switch_;
    private PIDController rotate_motor_PID_controller_;
    private RelativeEncoder rotate_encoder;

    private Pickup() {
        io_ = new PickupPeriodicIo();

        // Edit motor settings and add brushless motor type
        rotate_motor_ = new TalonFX(Constants.PickupConstants.ROTATE_MOTOR_ID);
        roller_motor_ = new TalonFX(Constants.PickupConstants.ROLLER_MOTOR_ID);
        algae_loaded_sensor_ = new TimeOfFlight(Constants.PickupConstants.PICKUP_LOADED_SENSOR_ID);
    }

    enum PickupStages {
        IDLE,
        ROTATE_IN,
        ROTATE_OUT,
        PICKUP,
    }

    // Singleton pattern
    private static Pickup pickup_instance_ = null;

    public static Pickup getInstance() {
        if (pickup_instance_ == null) {
            pickup_instance_ = new Pickup();
        }
        return pickup_instance_;
    }

    /**
     * Reads all sensors and stores periodic data
     */
    public void readPeriodicInputs(double timestamp) {
        io_.algae_distance_ = algae_loaded_sensor_.getRange();
    }

    /**
     * Computes updated outputs for the actuators
     */
    public void updateLogic(double timestamp) {
        switch (io_.pickup_stage_) {
            case IDLE:
                setIdlePosition();
                break;
            case ROTATE_IN:
                setIdlePosition();
                break;
            case ROTATE_OUT:
                setPickupPosition();
                break;
            case PICKUP:
                pickupAlgae();
                io_.target_degrees_ = Constants.PickupConstants.ROTATE_OUT_POSITION;
                break;
            default:
                setIdlePosition();
                stopRotateMotors();
                stopRollerMotors();
                break;
        }
    }

    /**
     * Writes the periodic outputs to actuators (motors and etc...)
     */
    public void writePeriodicOutputs(double timestamp) {
        roller_motor_.set(io_.roller_speed_);
        rotate_motor_.setControl(io_.rotate_speed_);

        rotate_motor_.set(rotate_motor_PID_controller_.calculate(rotate_encoder.getPosition(),
                Constants.PickupConstants.ROTATE_IN_POSITION));
        rotate_motor_.set(rotate_motor_PID_controller_.calculate(rotate_encoder.getPosition(),
                Constants.PickupConstants.ROTATE_OUT_POSITION));

        rotate_motor_PID_controller_ = new PIDController(0, 0, 0);
        rotate_motor_PID_controller_.setTolerance(5, 10);
        // rotate_motor_PID_controller_.enableContinuousInput(0, 180);
        // Clamps the controller output to between 0 and 0.5
        // MathUtil.clamp(rotate_motor_PID_controller_.calculate(rotate_encoder.getPosition(),
        // /*setpoint*/), 0, 0.5);
        // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/pidcontroller.html

    }

    /**
     * Outputs all logging information to the SmartDashboard
     */
    public void outputTelemetry(double timestamp) {
    }

    /**
     * Called to reset and configure the subsystem
     */
    public void reset() {
        stopRollerMotors();
        setIdlePosition();
        stopRotateMotors();
    }

    public void setPickupStage(PickupStages mode) {
        io_.pickup_stage_ = mode;
    }

    public void setIdlePosition() {
        while (io_.is_arm_at_idle_position_ == false) {
            io_.rotate_speed_ = Constants.PickupConstants.ROTATE_IN_SPEED;
        }
        stopRotateMotors();
        stopRollerMotors();
    }

    public void setPickupPosition() {
        while (io_.is_arm_at_pickup_position_ == false) {
            rotateOut();
        }
        stopRotateMotors();
        stopRollerMotors();
    }

    private void pickupAlgae() {

        setPickupPosition();
        while (io_.is_algae_loaded_ == false) {
            rotateIn();
        }
        stopRotateMotors();
        stopRollerMotors();
    }

    private void rotateIn() {
        io_.rotate_speed_ = Constants.PickupConstants.ROTATE_IN_SPEED;
    }

    private void rotateOut() {
        io_.rotate_speed_ = Constants.PickupConstants.ROTATE_OUT_SPEED;
    }

    private void stopRollerMotors() {
        io_.roller_speed_ = 0.0;
    }

    private void stopRotateMotors() {
        io_.rotate_speed_ = 0.0;
    }

    private void isArmAtIdlePosition() {
        io_.is_arm_at_idle_position_ = rotate_idle_switch_.get();
    }

    private void isArmAtPickupPosition() {
        io_.is_arm_at_pickup_position_ = rotate_pickup_switch_.get();
    }

    private void getAlgaeDistance() {
        io_.algae_distance_ = algae_loaded_sensor_.getRange();
    }

    private void isAlgaeLoaded() {
        if (io_.algae_distance_ < Constants.PickupConstants.ALGAE_PICKUP_TOF_LOADED_DISTANCE) {
            io_.is_algae_loaded_ = true;
        } else {
            io_.is_algae_loaded_ = false;
        }
    }

    private boolean getAlgaeLoaded() {
        if (io_.algae_distance_ < Constants.PickupConstants.ALGAE_PICKUP_TOF_LOADED_DISTANCE) {
            return true;
        } else {
            return false;
        }
    }

    public class PickupPeriodicIo implements Logged {
        @Log.File
        public PickupStages pickup_stage_ = PickupStages.IDLE;
        @Log.File
        public double roller_speed_ = 0.0;
        @Log.File
        public double rotate_speed_ = 0.0;
        @Log.File
        public boolean is_arm_at_idle_position_ = false;
        @Log.File
        public boolean is_arm_at_pickup_position_ = false;
        @Log.File
        public double algae_distance_ = 0.0;
        @Log.File
        public boolean is_algae_loaded_ = false;
        @Log.File
        public double target_degrees_ = 0.0;
    }

    /**
     * Get logging object from subsystem
     */
    public Logged getLoggingObject() {
        return io_;
    }
}
