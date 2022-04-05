package org.ghrobotics.frc2022.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.ghrobotics.lib.sensor.PicoColorSensor;
import org.ghrobotics.lib.telemetry.MissionControl;
import static com.revrobotics.CANSparkMax.IdleMode;
import static com.revrobotics.CANSparkMax.MotorType;

public class Feeder extends SubsystemBase {
  // Motor Controllers
  private final CANSparkMax floor_leader_;
  private final CANSparkMax wall_leader_;

  // Sensors
  private final PicoColorSensor color_sensors_;

  // IO
  private final PeriodicIO io_ = new PeriodicIO();

  /**
   * Constructs an instance of the Feeder subsystem. Only one instance of this subsystem should
   * be created in the main Robot class and references to this instance should be passed around
   * by the robot code.
   */
  public Feeder() {
    // Initialize motor controllers.
    floor_leader_ = new CANSparkMax(Constants.kFloorLeaderId, MotorType.kBrushless);
    floor_leader_.restoreFactoryDefaults();
    floor_leader_.setIdleMode(IdleMode.kBrake);
    floor_leader_.enableVoltageCompensation(12);
    floor_leader_.setSmartCurrentLimit(Constants.kCurrentLimit);
    floor_leader_.setInverted(false);

    wall_leader_ = new CANSparkMax(Constants.kWallLeaderId, MotorType.kBrushless);
    wall_leader_.restoreFactoryDefaults();
    wall_leader_.setIdleMode(IdleMode.kBrake);
    wall_leader_.enableVoltageCompensation(12);
    wall_leader_.setSmartCurrentLimit(Constants.kCurrentLimit);
    wall_leader_.setInverted(true);

    // Initialize sensors.
    color_sensors_ = new PicoColorSensor();

    // Add telemetry.
    MissionControl.addBoolean("feeder/lower_sensor", () -> io_.lower_sensor);
    MissionControl.addBoolean("feeder/upper_sensor", () -> io_.upper_sensor);
    MissionControl.addDouble("feeder/floor_supply_current", () -> io_.floor_supply_current);
    MissionControl.addDouble("feeder/wall_supply_current", () -> io_.wall_supply_current);
  }

  /**
   * This method runs periodically every 20 ms. Here, all sensor values are read and all motor
   * outputs should be set.
   */
  @Override
  public void periodic() {
    // Read inputs.
    io_.lower_sensor = color_sensors_.getProximity0() > Constants.kLowerProximityThreshold;
    io_.upper_sensor = color_sensors_.getProximity1() > Constants.kUpperProximityThreshold;

    io_.floor_supply_current = floor_leader_.getOutputCurrent();
    io_.wall_supply_current = wall_leader_.getOutputCurrent();

    // Write outputs.
    floor_leader_.set(io_.floor_demand);
    wall_leader_.set(io_.wall_demand);
  }

  public void setFloorPercent(double value) {
    io_.floor_demand = value;
  }

  public void setWallPercent(double value) {
    io_.wall_demand = value;
  }

  /**
   * Returns whether the lower sensor is tripped.
   *
   * @return Whether the lower sensor is tripped.
   */
  public boolean getLowerSensor() {
    return io_.lower_sensor;
  }

  /**
   * Returns whether the upper sensor is tripped.
   *
   * @return Whether the upper sensor is tripped.
   */
  public boolean getUpperSensor() {
    return io_.upper_sensor;
  }

  public static class PeriodicIO {
    boolean lower_sensor;
    boolean upper_sensor;

    double floor_supply_current;
    double wall_supply_current;

    double floor_demand;
    double wall_demand;
  }

  public static class Constants {
    // Motor Controllers
    public static final int kFloorLeaderId = 12;
    public static final int kWallLeaderId = 13;

    // Current Limits
    public static final int kCurrentLimit = 30;

    // Thresholds
    public static final int kUpperProximityThreshold = 125;
    public static final int kLowerProximityThreshold = 275;
  }
}
