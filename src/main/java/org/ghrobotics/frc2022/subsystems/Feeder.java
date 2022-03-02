package org.ghrobotics.frc2022.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static com.revrobotics.CANSparkMax.IdleMode;
import static com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Feeder extends SubsystemBase {
  // Motor Controllers
  private final CANSparkMax conveyor_leader_;
  private final CANSparkMax bridge_leader_;

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
    conveyor_leader_ = new CANSparkMax(Constants.kFeederId, MotorType.kBrushless);
    conveyor_leader_.restoreFactoryDefaults();
    conveyor_leader_.setIdleMode(IdleMode.kBrake);
    conveyor_leader_.enableVoltageCompensation(12);
    conveyor_leader_.setInverted(false);

    bridge_leader_ = new CANSparkMax(Constants.kBridgeId, MotorType.kBrushless);
    bridge_leader_.restoreFactoryDefaults();
    bridge_leader_.setIdleMode(IdleMode.kBrake);
    bridge_leader_.enableVoltageCompensation(12);
    bridge_leader_.setInverted(false);

    //Initialize sensors
    intake_sensor_ = new AnalogInput(Constants.kIntakeSensorId);
    exit_sensor_ = new AnalogInput(Constants.kExitSensorId);
  }

  public void periodic() {
    // Write outputs.
    conveyor_leader_.set(io_.demand);
    bridge_leader_.set(io_.demand);
  }

  /**
   * Sets the % output on the feeder.
   *
   * @param value The % output in [-1, 1].
   */
  public void setPercent(double value) {
    io_.demand = value;
  }

  public static class PeriodicIO {
    // Outputs
    double demand;
  }

  public static class Constants {
    //Motor Controllers
    public static final int kFeederId = 12;
    public static final int kBridgeId = 10;

    //Sensors
    public static final int kIntakeSensorId = 0;
    public static final int kExitSensorId = 0;
  }
}