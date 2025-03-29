package frc.mw_lib.drivers;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.hal.SimDouble;

public class SimTof {

  private TimeOfFlight tof_;
  private SimDevice dev_;
  private SimDouble sim_dist_;

  public SimTof(int sensor_id) {
    dev_ = SimDevice.create("TOF Sensor", sensor_id);
    if (dev_ == null) {
      tof_ = new TimeOfFlight(sensor_id);
    } else {
      sim_dist_ = dev_.createDouble("Distance", Direction.kInput, 1000.0);
    }
  }

  public double getRange() {
    if (dev_ == null) {
      return tof_.getRange();
    } else {
      return sim_dist_.get();
    }
  }

  public void setRangingMode(RangingMode mode, double sampleTime) {
    if (dev_ == null) {
      tof_.setRangingMode(mode, sampleTime);
    }
  }
}
