package org.ghrobotics.frc2022.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static com.revrobotics.CANSparkMax.IdleMode;
import static com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Intake extends SubsystemBase {
  // Motor Controllers
  private final CANSparkMax intake_leader_;
  private final CANSparkMax bridge_leader_;
  private final CANSparkMax bridge_follower_;
  private final CANSparkMax feeder_floor_leader_;
  private final CANSparkMax feeder_wall_leader_;

  // Pneumatics
  private final DoubleSolenoid pivot_;

  // Sensors
  private final AnalogInput floor_sensor_;
  private final AnalogInput wall_sensor_;

  // IO
  private final PeriodicIO io_ = new PeriodicIO();

  /**
   * Constructs an instance of the Intake subsystem. Only one instance of this subsystem should
   * be created in the main Robot class and references to this instance should be passed around
   * the robot code.
   */
  public Intake() {
    // Initialize motor controllers.
    intake_leader_ = new CANSparkMax(Constants.kIntakeLeaderId, MotorType.kBrushless);
    intake_leader_.restoreFactoryDefaults();
    intake_leader_.setIdleMode(IdleMode.kCoast);
    intake_leader_.enableVoltageCompensation(12);
    intake_leader_.setInverted(true);

    bridge_leader_ = new CANSparkMax(Constants.kBridgeLeaderId, MotorType.kBrushless);
    bridge_leader_.restoreFactoryDefaults();
    bridge_leader_.setIdleMode(IdleMode.kCoast);
    bridge_leader_.enableVoltageCompensation(12);
    bridge_leader_.setInverted(false);

    bridge_follower_ = new CANSparkMax(Constants.kBridgeFollowerId, MotorType.kBrushless);
    bridge_follower_.restoreFactoryDefaults();
    bridge_follower_.setIdleMode(IdleMode.kCoast);
    bridge_follower_.enableVoltageCompensation(12);
    bridge_follower_.follow(bridge_leader_, true);

    feeder_floor_leader_ = new CANSparkMax(Constants.kFeederFloorId, MotorType.kBrushless);
    feeder_floor_leader_.restoreFactoryDefaults();
    feeder_floor_leader_.setIdleMode(IdleMode.kBrake);
    feeder_floor_leader_.enableVoltageCompensation(12);
    feeder_floor_leader_.setSmartCurrentLimit(Constants.kFeederCurrentLimit);
    feeder_floor_leader_.setInverted(false);

    feeder_wall_leader_ = new CANSparkMax(Constants.kFeederWallId, MotorType.kBrushless);
    feeder_wall_leader_.restoreFactoryDefaults();
    feeder_wall_leader_.setIdleMode(IdleMode.kBrake);
    feeder_wall_leader_.enableVoltageCompensation(12);
    feeder_wall_leader_.setSmartCurrentLimit(Constants.kFeederCurrentLimit);
    feeder_wall_leader_.setInverted(true);

    // Initialize pneumatics.
    pivot_ = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.kPivotForwardId,
        Constants.kPivotReverseId);

    // Initialize sensors.
    floor_sensor_ = new AnalogInput(Constants.kFeederFloorSensorId);
    wall_sensor_ = new AnalogInput(Constants.kFeederWallSensorId);
  }

  /**
   * This method runs periodically every 20 ms. Here, all sensor values are read and all motor
   * outputs should be set.
   */
  @Override
  public void periodic() {
    // Read inputs.
    io_.floor_sensor = floor_sensor_.getAverageVoltage() > Constants.kFeederFloorSensorVThreshold;
    io_.wall_sensor = wall_sensor_.getAverageVoltage() > Constants.kFeederWallSensorVThreshold;
    io_.intake_supply_current = intake_leader_.getOutputCurrent();
    io_.bridge_leader_supply_current = bridge_leader_.getOutputCurrent();
    io_.bridge_follower_supply_current = bridge_follower_.getOutputCurrent();
    io_.feeder_floor_supply_current = feeder_floor_leader_.getOutputCurrent();
    io_.feeder_wall_supply_current = feeder_wall_leader_.getOutputCurrent();

    // Write outputs.
    if (io_.wants_pneumatics_update) {
      io_.wants_pneumatics_update = false;
      pivot_.set(io_.pivot_value ? DoubleSolenoid.Value.kReverse : DoubleSolenoid.Value.kForward);
    }

    intake_leader_.set(io_.intake_bridge_demand);
    bridge_leader_.set(io_.intake_bridge_demand);
    feeder_floor_leader_.set(io_.feeder_floor_demand);
    feeder_wall_leader_.set(io_.feeder_wall_demand);
  }

  /**
   * Sets the % output on the intake.
   *
   * @param value The % output in [-1, 1].
   */
  public void setIntakePercent(double value) {
    io_.intake_bridge_demand = value;
  }

  /**
   * Sets the % output on the feeder floor.
   *
   * @param value The % output in [-1, 1].
   */
  public void setFloorPercent(double value) {
    io_.feeder_floor_demand = value;
  }

  /**
   * Sets the % output on the feeder wall.
   *
   * @param value The % output in [-1, 1].
   */
  public void setWallPercent(double value) {
    io_.feeder_wall_demand = value;
  }

  /**
   * Returns the state of the intake photoelectric sensor.
   *
   * @return The state of the intake photoelectric sensor; true if triggered.
   */
  public boolean getIntakeSensor() {
    return io_.floor_sensor;
  }

  /**
   * Returns the state of the exit photoelectric sensor.
   *
   * @return The state of the exit photoelectric sensor; true if triggered.
   */
  public boolean getExitSensor() {
    return io_.wall_sensor;
  }


  /**
   * Sets the intake pivot.
   *
   * @param value The pivot value; true if the intake should be extended.
   */
  public void setPivot(boolean value) {
    io_.wants_pneumatics_update = value != io_.pivot_value;
    io_.pivot_value = value;
  }

  public static class PeriodicIO {
    // Inputs
    boolean floor_sensor;
    boolean wall_sensor;
    double intake_supply_current;
    double bridge_leader_supply_current;
    double bridge_follower_supply_current;
    double feeder_floor_supply_current;
    double feeder_wall_supply_current;

    // Outputs
    double intake_bridge_demand;
    double feeder_floor_demand;
    double feeder_wall_demand;

    boolean pivot_value = false;
    boolean wants_pneumatics_update = true;
  }

  public static class Constants {
    // Motor Controllers
    public static final int kIntakeLeaderId = 9;
    public static final int kBridgeLeaderId = 10;
    public static final int kBridgeFollowerId = 11;
    public static final int kFeederFloorId = 12;
    public static final int kFeederWallId = 13;

    // Pneumatics
    public static final int kPivotForwardId = 5;
    public static final int kPivotReverseId = 6;

    // Sensors
    public static final int kFeederFloorSensorId = 2;
    public static final int kFeederWallSensorId = 3;

    // Current Limits
    public static final int kIntakeCurrentLimit = 80;
    public static final int kBridgeCurrentLimit = 20;
    public static final int kFeederCurrentLimit = 25;

    // Thresholds
    public static final double kFeederFloorSensorVThreshold = 1.28;
    public static final double kFeederWallSensorVThreshold = 2.7;
  }
}