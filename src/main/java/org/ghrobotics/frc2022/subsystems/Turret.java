package org.ghrobotics.frc2022.subsystems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static com.revrobotics.CANSparkMax.IdleMode;
import static com.revrobotics.CANSparkMax.MotorType;

public class Turret extends SubsystemBase {
  // Motor Controllers
  private final CANSparkMax leader_;

  // Sensors
  private final CANCoder encoder_;

  // Control
  private final ProfiledPIDController pid_controller_;
  private final SimpleMotorFeedforward feedforward_;

  // IO
  private OutputType output_type_ = OutputType.PERCENT;
  private final PeriodicIO io_ = new PeriodicIO();

  /**
   * Constructs an instance of the Turret subsystem. Only one instance of this subsystem should
   * be created in the main Robot class and references to this instance should be passed around
   * the robot code.
   */
  public Turret() {
    // Initialize motor controller.
    leader_ = new CANSparkMax(Constants.kLeaderId, MotorType.kBrushless);
    leader_.restoreFactoryDefaults();
    leader_.setIdleMode(IdleMode.kBrake);
    leader_.enableVoltageCompensation(12);
    leader_.setSmartCurrentLimit(Constants.kCurrentLimit);
    leader_.setInverted(false);

    // Initialize encoder.
    encoder_ = new CANCoder(Constants.kEncoderId);
    encoder_.configFactoryDefault();
    encoder_.configFeedbackCoefficient(
        2 * Math.PI / Constants.kGearRatio / Constants.kEncoderResolution, "rad",
        SensorTimeBase.PerSecond);
    encoder_.configSensorDirection(true);
    encoder_.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_10Ms);
    encoder_.configVelocityMeasurementWindow(8);
    encoder_.configMagnetOffset(Constants.kEncoderMagnetOffset);
    encoder_.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
    encoder_.configSensorInitializationStrategy(
        SensorInitializationStrategy.BootToAbsolutePosition);

    // Initialize PID controller.
    pid_controller_ = new ProfiledPIDController(Constants.kP, 0, 0,
        new TrapezoidProfile.Constraints(Constants.kMaxVelocity, Constants.kMaxAcceleration));

    // Initialize feedforward.
    feedforward_ = new SimpleMotorFeedforward(Constants.kS, Constants.kV, Constants.kA);
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

    // Write outputs.
    switch (output_type_) {
      case PERCENT:
        // Send the percent output value directly to the motor controller.
        leader_.set(io_.demand);
        break;
      case PROFILE:
        // Calculate motor controller voltage from feedback and feedforward.
        double feedback = pid_controller_.calculate(io_.position);
        TrapezoidProfile.State setpoint = pid_controller_.getSetpoint();
        double feedforward = feedforward_.calculate(setpoint.velocity,
            (setpoint.velocity - io_.velocity) / 0.02);

        leader_.setVoltage(feedback + feedforward);
        break;
    }
  }

  /**
   * Sets the % output on the turret.
   *
   * @param value The % output in [-1, 1].
   */
  public void setPercent(double value) {
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

    // Outputs
    double demand;
  }

  public static class Constants {
    // Motor Controllers
    public static final int kLeaderId = 5;

    // Sensors
    public static final int kEncoderId = 18;

    // Current Limits
    public static final int kCurrentLimit = 30;

    // Hardware
    public static final double kGearRatio = 150.0 / 16.0 * 7.0;
    public static final double kMinAngle = 0;
    public static final double kMaxAngle = 2 * Math.PI;
    public static final double kEncoderMagnetOffset = 0;
    public static final double kEncoderResolution = 4096;

    // Control
    public static final double kS = 0.0;
    public static final double kV = 0.0;
    public static final double kA = 0.0;
    public static final double kP = 0.0;
    public static final double kMaxVelocity = 4 * Math.PI;
    public static final double kMaxAcceleration = 4 * Math.PI;
  }
}
