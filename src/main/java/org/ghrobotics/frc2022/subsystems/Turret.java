package org.ghrobotics.frc2022.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.ghrobotics.frc2022.RobotState;
import static com.revrobotics.CANSparkMax.IdleMode;
import static com.revrobotics.CANSparkMax.MotorType;

public class Turret extends SubsystemBase {
  // Robot State
  private final RobotState robot_state_;

  // Motor Controllers
  private final CANSparkMax leader_;

  // Sensors
  private final RelativeEncoder encoder_;
  private final DigitalInput hall_sensor_;

  // Control
  private final ProfiledPIDController pid_controller_;
  private final SimpleMotorFeedforward feedforward_;
  private double last_velocity_setpoint_ = 0;
  private boolean reset_pid_ = true;

  // Simulation
  private final DCMotorSim physics_sim_;
  private final SimDeviceSim leader_sim_;

  // IO
  private OutputType output_type_ = OutputType.PERCENT;
  private final PeriodicIO io_ = new PeriodicIO();

  /**
   * Constructs an instance of the Turret subsystem. Only one instance of this subsystem should
   * be created in the main Robot class and references to this instance should be passed around
   * the robot code.
   *
   * @param robot_state Reference to the global robot state instance.
   */
  public Turret(RobotState robot_state) {
    // Store reference to robot state.
    robot_state_ = robot_state;

    // Initialize motor controller.
    leader_ = new CANSparkMax(Constants.kLeaderId, MotorType.kBrushless);
    leader_.restoreFactoryDefaults();
    leader_.setIdleMode(IdleMode.kBrake);
    leader_.enableVoltageCompensation(12);
    leader_.setSmartCurrentLimit(Constants.kCurrentLimit);
    leader_.setInverted(false);

    // Initialize encoder.
    encoder_ = leader_.getEncoder();
    encoder_.setPositionConversionFactor(2 * Math.PI / Constants.kGearRatio);
    encoder_.setVelocityConversionFactor(2 * Math.PI / Constants.kGearRatio / 60);

    // Initialize hall sensor.
    hall_sensor_ = new DigitalInput(Constants.kLimitSwitchId);

    // Initialize PID controller.
    pid_controller_ = new ProfiledPIDController(Constants.kP, 0, Constants.kD,
        new TrapezoidProfile.Constraints(Constants.kMaxVelocity, Constants.kMaxAcceleration));

    // Initialize feedforward.
    feedforward_ = new SimpleMotorFeedforward(Constants.kS, Constants.kV, Constants.kA);

    // Initialize simulation objects.
    physics_sim_ = new DCMotorSim(DCMotor.getNEO(1), Constants.kGearRatio, Constants.kMOI);
    leader_sim_ = new SimDeviceSim("SPARK MAX [" + Constants.kLeaderId + "]");
  }

  /**
   * This method runs periodically every 20 ms. Here, all sensor values are read and all motor
   * outputs should be set.
   */
  @Override
  public void periodic() {
    // Read inputs.
    io_.position = encoder_.getPosition();
    io_.velocity = encoder_.getVelocity();
    io_.hall_sensor = !hall_sensor_.get();

    // Update robot state.
    robot_state_.updateTurretAngle(new Rotation2d(io_.position));

    // Reset PID controller if we have to.
    if (reset_pid_) {
      reset_pid_ = false;
      pid_controller_.reset(io_.position, io_.velocity);
    }

    // Write outputs.
    switch (output_type_) {
      case PERCENT:
        // Send the percent output value directly to the motor controller.
        leader_.set(io_.demand);

        // Update simulated inputs.
        if (RobotBase.isSimulation())
          leader_sim_.getDouble("Applied Output").set(io_.demand * 12);
        break;
      case PROFILE:
        // Calculate motor controller voltage from feedback and feedforward.
        double feedback = pid_controller_.calculate(io_.position);
        TrapezoidProfile.State setpoint = pid_controller_.getSetpoint();
        double feedforward = feedforward_.calculate(setpoint.velocity,
            (setpoint.velocity - last_velocity_setpoint_) / 0.02);

        // Store last velocity setpoint.
        last_velocity_setpoint_ = setpoint.velocity;

        // Set voltage.
        leader_.setVoltage(feedback + feedforward);
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
    physics_sim_.setInputVoltage(-leader_.getAppliedOutput());

    // Update physics sim forward in time.
    physics_sim_.update(0.02);

    // Set encoder inputs.
    leader_sim_.getDouble("Position").set(physics_sim_.getAngularPositionRad());
    leader_sim_.getDouble("Velocity").set(physics_sim_.getAngularVelocityRadPerSec());
  }

  /**
   * Sets the idle mode on the turret.
   *
   * @param mode The desired idle mode.
   */
  public void setIdleMode(IdleMode mode) {
    leader_.setIdleMode(mode);
  }

  /**
   * Sets the % output on the turret.
   *
   * @param value The % output in [-1, 1].
   */
  public void setPercent(double value) {
    last_velocity_setpoint_ = 0;
    reset_pid_ = true;
    output_type_ = OutputType.PERCENT;
    io_.demand = value;
  }

  /**
   * Sets the turret goal position and velocity.
   *
   * @param position The goal position in radians.
   * @param velocity The goal velocity in radians per second.
   */
  public void setGoal(double position, double velocity) {
    output_type_ = OutputType.PROFILE;
    pid_controller_.setGoal(new TrapezoidProfile.State(constrainSetpoint(position), velocity));
  }

  /**
   * Returns the current turret position in radians.
   *
   * @return The current turret position in radians.
   */
  public double getPosition() {
    return io_.position;
  }

  /**
   * Returns the current turret velocity in radians per second.
   *
   * @return The current turret velocity in radians per second.
   */
  public double getVelocity() {
    return io_.velocity;
  }

  public boolean getLimitSwitch() {
    SmartDashboard.putBoolean("Lim", io_.hall_sensor);
    return io_.hall_sensor;
  }

  /**
   * Constrains a given setpoint within the range of the turret.
   *
   * @param setpoint The angular setpoint in radians.
   * @return The constrained setpoint.
   */
  private static double constrainSetpoint(double setpoint) {
    while (setpoint >= Constants.kMaxAngle)
      setpoint -= 2 * Math.PI;
    while (setpoint < Constants.kMinAngle)
      setpoint += 2 * Math.PI;

    return setpoint;
  }

  public enum OutputType {
    PERCENT, PROFILE
  }

  public static class PeriodicIO {
    // Inputs
    double position;
    double velocity;
    boolean hall_sensor;

    // Outputs
    double demand;
  }

  public static class Constants {
    // Motor Controllers
    public static final int kLeaderId = 5;

    // Sensors
    public static final int kLimitSwitchId = 9;

    // Current Limits
    public static final int kCurrentLimit = 30;

    // Hardware
    public static final double kMinAngle = 0;
    public static final double kMaxAngle = 2 * Math.PI;
    public static final double kGearRatio = 7.0 * 150 / 16.0;
    public static final double kMOI = 0.1764;

    // Control
    public static final double kS = 0.22;
    public static final double kV = 1.2801;
    public static final double kA = 0.069845;
    public static final double kP = 2.54;
    public static final double kD = 0.254;
    public static final double kMaxVelocity = 4 * Math.PI;
    public static final double kMaxAcceleration = 3 * Math.PI;
  }
}
