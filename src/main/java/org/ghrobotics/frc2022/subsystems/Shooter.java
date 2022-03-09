package org.ghrobotics.frc2022.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderSimCollection;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static com.revrobotics.CANSparkMax.IdleMode;
import static com.revrobotics.CANSparkMax.MotorType;

public class Shooter extends SubsystemBase {
  // Motor Controllers
  private final CANSparkMax leader_;
  private final CANSparkMax follower_;

  // Sensors
  private final CANCoder encoder_;

  // Control
  private final BangBangController bang_bang_controller_;
  private final SimpleMotorFeedforward feedforward_;
  private double last_velocity_setpoint_ = 0;

  // Simulation
  private final FlywheelSim physics_sim_;
  private final SimDeviceSim leader_sim_;
  private final CANCoderSimCollection encoder_sim_;

  // IO
  private OutputType output_type_ = OutputType.PERCENT;
  private final PeriodicIO io_ = new PeriodicIO();

  /**
   * Constructs an instance of the Shooter subsystem. Only one instance of this subsystem should
   * be created in the main Robot class and references to this instance should be passed around
   * the robot code.
   */
  public Shooter() {
    // Initialize motor controllers.
    leader_ = new CANSparkMax(Constants.kLeaderId, MotorType.kBrushless);
    leader_.restoreFactoryDefaults();
    leader_.setIdleMode(IdleMode.kCoast);
    leader_.enableVoltageCompensation(12);
    leader_.setSmartCurrentLimit(Constants.kCurrentLimit);
    leader_.setInverted(true);

    follower_ = new CANSparkMax(Constants.kFollowerId, MotorType.kBrushless);
    follower_.restoreFactoryDefaults();
    follower_.setIdleMode(IdleMode.kCoast);
    follower_.enableVoltageCompensation(12);
    follower_.setSmartCurrentLimit(Constants.kCurrentLimit);
    follower_.follow(leader_, true);

    // Initialize encoder.
    encoder_ = new CANCoder(Constants.kEncoderId);
    encoder_.configFactoryDefault();
    encoder_.configFeedbackCoefficient(
        2 * Math.PI / Constants.kEncoderGearRatio / Constants.kEncoderResolution,
        "rad", SensorTimeBase.PerSecond);
    encoder_.configSensorDirection(true);
    encoder_.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_2Ms);
    encoder_.configVelocityMeasurementWindow(4);

    // Initialize bang-bang controller.
    bang_bang_controller_ = new BangBangController(Constants.kErrorTolerance);

    // Initialize feedforward.
    feedforward_ = new SimpleMotorFeedforward(Constants.kS, Constants.kV, Constants.kA);

    // Initialize simulation objects.
    physics_sim_ = new FlywheelSim(
        LinearSystemId.identifyVelocitySystem(Constants.kV, Constants.kA),
        DCMotor.getNEO(2), Constants.kGearRatio);
    leader_sim_ = new SimDeviceSim("SPARK MAX [" + Constants.kLeaderId + "]");
    encoder_sim_ = encoder_.getSimCollection();
  }

  /**
   * This method runs periodically every 20 ms. Here, all sensor values are read and all motor
   * outputs should be set.
   */
  @Override
  public void periodic() {
    // Read inputs.
    io_.velocity = encoder_.getVelocity();

    // Write outputs.
    switch (output_type_) {
      case PERCENT:
        // Send the percent output value directly to the motor controller.
        leader_.set(io_.demand);

        // Set simulated inputs.
        if (RobotBase.isSimulation()) {
          leader_sim_.getDouble("Applied Output").set(io_.demand * 12);
        }
        break;
      case VELOCITY:
        // Calculate motor controller voltage from feedback and feedforward.
        double feedback = bang_bang_controller_.calculate(io_.velocity);
        double setpoint = bang_bang_controller_.getSetpoint();
        double feedforward = feedforward_.calculate(setpoint,
            (setpoint - last_velocity_setpoint_) / 0.02);

        // Store last velocity setpoint.
        last_velocity_setpoint_ = setpoint;

        // We multiply by 0.9 to avoid overshoot. Divide by 12 to change voltage to percent.
        // See: https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/bang-bang.html#combining-bang-bang-control-with-feedforward
        leader_.setVoltage(feedback * 12 + 0.9 * feedforward);
        break;
    }
  }

  /**
   * This method runs periodically every 20 ms in simulation. Here, the physics model should be
   * updated and simulated sensor outputs should be set.
   */
  @Override
  public void simulationPeriodic() {
    // Update physics sim with inputs.
    // Note: a bug with REV simulation causes getAppliedOutput() to return voltage instead of
    // duty cycle, which is why we are not multiplying by 12.
    physics_sim_.setInputVoltage(leader_.getAppliedOutput());

    // Update physics sim forward in time.
    physics_sim_.update(0.02);

    // Update encoder velocity.
    encoder_sim_.setVelocity(
        (int) (physics_sim_.getAngularVelocityRadPerSec() * Constants.kEncoderGearRatio *
            Constants.kEncoderResolution / 2 / Math.PI / 10));
  }

  /**
   * Sets the % output on the shooter.
   *
   * @param value The % output in [-1, 1].
   */
  public void setPercent(double value) {
    last_velocity_setpoint_ = 0;
    output_type_ = OutputType.PERCENT;
    io_.demand = value;
  }

  /**
   * Sets the shooter velocity.
   *
   * @param value The velocity in radians per second.
   */
  public void setVelocity(double value) {
    output_type_ = OutputType.VELOCITY;
    bang_bang_controller_.setSetpoint(value);
  }

  /**
   * Sets the shooter velocity in RPM.
   *
   * @param value The velocity in RPM.
   */
  public void setRPM(double value) {
    setVelocity(Units.rotationsPerMinuteToRadiansPerSecond(value));
  }

  /**
   * Returns whether the shooter is at the goal (velocity setpoint).
   *
   * @return Whether the shooter is at the goal (velocity setpoint).
   */
  public boolean atGoal() {
    return bang_bang_controller_.atSetpoint();
  }

  /**
   * Returns the current shooter velocity in radians per second.
   *
   * @return The current shooter velocity in radians per second.
   */
  public double getVelocity() {
    return io_.velocity;
  }

  public enum OutputType {
    PERCENT, VELOCITY
  }

  public static class PeriodicIO {
    // Inputs
    double velocity;

    // Outputs
    double demand;
  }

  public static class Constants {
    // Motor Controllers
    public static final int kLeaderId = 6;
    public static final int kFollowerId = 7;

    // Sensors
    public static final int kEncoderId = 19;

    // Current Limits
    public static final int kCurrentLimit = 80;

    // Hardware
    public static final double kGearRatio = 0.5;
    public static final double kEncoderGearRatio = 30.0 / 18.0;
    public static final double kEncoderResolution = 4096;
    public static final double kWheelRadius = 0.0508;

    // Control
    public static final double kS = 0.64101;
    public static final double kV = 0.013676;
    public static final double kA = 0.0029748;
    public static final double kErrorTolerance = Units.rotationsPerMinuteToRadiansPerSecond(300);
  }
}
