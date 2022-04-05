package org.ghrobotics.frc2022.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.ghrobotics.lib.telemetry.MissionControl;

public class PressureSensor extends SubsystemBase {
  // Sensor
  private final
  AnalogInput sensor_;

  // IO
  private final PeriodicIO io_ = new PeriodicIO();

  /**
   * Represents the subsystem for a pressure sensor wired into the roboRIO.
   */
  public PressureSensor() {
    // Initialize pressure sensor.
    sensor_ = new AnalogInput(Constants.kSensorId);

    // Add telemetry.
    MissionControl.addDouble("air_pressure", () -> io_.pressure);
  }

  /**
   * Runs periodically every 20 ms to poll the current air pressure.
   */
  @Override
  public void periodic() {
    io_.pressure = 250 * (sensor_.getAverageVoltage() / 5.0) - 25;
  }

  public static class PeriodicIO {
    double pressure;
  }

  public static class Constants {
    public static final int kSensorId = 0;
  }
}
