package org.ghrobotics.frc2022.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.Arrays;

public class Limelight {
  // NetworkTable with keys for data.
  private final NetworkTable table_;

  // IO
  private final PeriodicIO io_ = new PeriodicIO();

  // Creates a new Limelight instance with the given table name.
  // See https://docs.limelightvision.io/en/latest/networktables_api.html
  public Limelight(String name) {
    table_ = NetworkTableInstance.getDefault().getTable(name);
  }

  // Do we have a target?
  public boolean hasTarget() {
    return io_.tv != 0;
  }

  // Horizontal offset to target.
  public double getTx() {
    return io_.tx;
  }

  // Vertical offset to target.
  public double getTy() {
    return io_.ty;
  }

  // Area of target.
  public double getArea() {
    return io_.ta;
  }

  // Skew of target.
  public double getSkew() {
    return io_.ts;
  }

  // Pipeline latency (ms).
  public double getLatency() {
    // Add 11 ms for image capture latency.
    return 11 + io_.tl;
  }

  // Target corners.
  public double[] getTargetCorners() {
    // Create a copy to prevent returning reference to io_.tcornxy to caller.
    return Arrays.copyOf(io_.tcornxy, io_.tcornxy.length);
  }

  // Setter for LED mode.
  public void setLED(LEDMode mode) {
    io_.led_mode = mode.ordinal();
  }

  // Setter for pipeline.
  public void setPipeline(int pipeline) {
    io_.pipeline = pipeline;
  }

  // Runs periodically every 20 ms.
  public void periodic() {
    // Read inputs.
    io_.tv = table_.getEntry("tv").getDouble(0);
    io_.tx = table_.getEntry("tx").getDouble(0);
    io_.ty = table_.getEntry("ty").getDouble(0);
    io_.ta = table_.getEntry("ta").getDouble(0);
    io_.ts = table_.getEntry("ts").getDouble(0);
    io_.tl = table_.getEntry("tl").getDouble(0);
    io_.tcornxy = table_.getEntry("tcornxy").getDoubleArray(new double[]{});

    // Write outputs.
    table_.getEntry("ledMode").setDouble(io_.led_mode);
    table_.getEntry("pipeline").setDouble(io_.pipeline);
  }

  // LED Modes
  enum LEDMode {
    PIPELINE, OFF, BLINK, ON
  }

  // IO Structure
  static class PeriodicIO {
    // Measurements (inputs)
    double tv;
    double tx;
    double ty;
    double ta;
    double ts;
    double tl;
    double[] tcornxy;

    // Commands (outputs)
    double led_mode;
    double pipeline;
  }
}
