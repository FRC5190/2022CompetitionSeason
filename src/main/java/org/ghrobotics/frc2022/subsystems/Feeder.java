package org.ghrobotics.frc2022.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static com.revrobotics.CANSparkMax.IdleMode;
import static com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Feeder extends SubsystemBase {
  // Motor Controllers
  private final CANSparkMax feeder_floor_;
  private final CANSparkMax feeder_wall_;

  // Sensors
  private final AnalogInput intake_sensor_;
  private final AnalogInput exit_sensor_;

  // IO
  private final PeriodicIO io_ = new PeriodicIO();

  /**
   * Constructs an instance of the Feeder subsystem. Only one instance of this subsystem should
   * be created in the main Robot class and references to this instance should be passed around
   * the robot code.
   */
  public Feeder() {
    // Initialize motor controllers.
    feeder_floor_ = new CANSparkMax(Constants.kFeederFloorId, MotorType.kBrushless);
    feeder_floor_.restoreFactoryDefaults();
    feeder_floor_.setIdleMode(IdleMode.kBrake);
    feeder_floor_.enableVoltageCompensation(12);
    feeder_floor_.setSmartCurrentLimit(Constants.kCurrentLimit);
    feeder_floor_.setInverted(false);

    feeder_wall_ = new CANSparkMax(Constants.kFeederWallId, MotorType.kBrushless);
    feeder_wall_.restoreFactoryDefaults();
    feeder_wall_.setIdleMode(IdleMode.kBrake);
    feeder_wall_.enableVoltageCompensation(12);
    feeder_wall_.setSmartCurrentLimit(Constants.kCurrentLimit);
    feeder_wall_.setInverted(true);

    // Initialize sensors.
    intake_sensor_ = new AnalogInput(Constants.kIntakeSensorId);
    exit_sensor_ = new AnalogInput(Constants.kExitSensorId);
  }

  public void periodic() {
    // Read inputs.
    io_.intake_sensor = intake_sensor_.getAverageVoltage() > Constants.kIntakeSensorVThreshold;
    io_.exit_sensor = exit_sensor_.getAverageVoltage() > Constants.kExitSensorVThreshold;

    // Write outputs.
    feeder_floor_.set(io_.floor_demand);
    feeder_wall_.set(io_.wall_demand);
  }

  /**
   * Sets the % output on the feeder floor.
   *
   * @param value The % output in [-1, 1].
   */
  public void setFloorPercent(double value) {
    io_.floor_demand = value;
  }

  /**
   * Sets the % output on the feeder wall.
   *
   * @param value The % output in [-1, 1].
   */
  public void setWallPercent(double value) {
    io_.wall_demand = value;
  }

  /**
   * Returns the state of the intake photoelectric sensor.
   *
   * @return The state of the intake photoelectric sensor; true if triggered.
   */
  public boolean getIntakeSensor() {
    return io_.intake_sensor;
  }

  /**
   * Returns the state of the exit photoelectric sensor.
   *
   * @return The state of the exit photoelectric sensor; true if triggered.
   */
  public boolean getExitSensor() {
    return io_.exit_sensor;
  }

  public static class PeriodicIO {
    // Inputs
    boolean intake_sensor;
    boolean exit_sensor;

    // Outputs
    double floor_demand;
    double wall_demand;
  }

  public static class Constants {
    // Motor Controllers
    public static final int kFeederFloorId = 12;
    public static final int kFeederWallId = 13;

    // Sensors
    public static final int kIntakeSensorId = 0;
    public static final int kExitSensorId = 1;

    // Current Limits
    public static final int kCurrentLimit = 25;

    // Thresholds
    public static final double kIntakeSensorVThreshold = 1.25;
    public static final double kExitSensorVThreshold = 1.25;
  }
}