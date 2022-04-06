package org.ghrobotics.frc2022.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.ghrobotics.frc2022.RobotState;
import org.ghrobotics.lib.telemetry.MissionControl;
import static com.revrobotics.CANSparkMax.IdleMode;
import static com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Hood extends SubsystemBase {
  // Robot State
  private final RobotState robot_state_;

  // Motor Controllers
  private final CANSparkMax leader_;

  // Sensors
  private final AnalogEncoder encoder_;

  // Control
  private final ProfiledPIDController pid_controller_;
  private final ArmFeedforward feedforward_;
  private double last_velocity_setpoint_ = 0;
  private boolean reset_pid_ = true;

  // IO
  private OutputType output_type_ = OutputType.PERCENT;
  private final PeriodicIO io_ = new PeriodicIO();

  /**
   * Constructs an instance of the Hood subsystem. Only one instance of this subsystem should
   * be created in the main Robot class and references to this instance should be passed around
   * the robot code.
   *
   * @param robot_state Reference to the global robot state instance.
   */
  public Hood(RobotState robot_state) {
    // Store reference to robot state.
    robot_state_ = robot_state;

    // Initialize motor controller.
    leader_ = new CANSparkMax(Constants.kLeaderId, MotorType.kBrushless);
    leader_.restoreFactoryDefaults();
    leader_.setIdleMode(IdleMode.kBrake);
    leader_.enableVoltageCompensation(12);
    leader_.setSmartCurrentLimit(Constants.kCurrentLimit);
    leader_.setInverted(true);

    // Initialize encoder.
    encoder_ = new AnalogEncoder(Constants.kEncoderId);

    // Initialize PID controller.
    pid_controller_ = new ProfiledPIDController(Constants.kP, 0, 0,
        new TrapezoidProfile.Constraints(Constants.kMaxVelocity, Constants.kMaxAcceleration));
    pid_controller_.setTolerance(Constants.kTolerance);

    // Initialize feedforward.
    feedforward_ = new ArmFeedforward(Constants.kS, Constants.kG, Constants.kV, Constants.kA);

    // Add telemetry.
    MissionControl.addDouble("hood/raw_enc", () -> io_.raw_enc);
    MissionControl.addDouble("hood/position", () -> io_.position);
    MissionControl.addDouble("hood/supply_current", () -> io_.supply_current);
  }

  /**
   * This method runs periodically every 20 ms. Here, all sensor values are read and all motor
   * outputs should be set.
   */
  @Override
  public void periodic() {
    // Read inputs.
    io_.raw_enc = encoder_.get();
    io_.position = Constants.kEncoderSlope * (io_.raw_enc - Constants.kMinEncoderValue)
        + Constants.kMinAngle;
    io_.supply_current = leader_.getOutputCurrent();

    SmartDashboard.putNumber("Hood Enc Raw", encoder_.get());

    // Update robot state.
    robot_state_.updateHoodAngle(new Rotation2d(io_.position));

    // Reset PID controller if we have to.
    if (reset_pid_) {
      reset_pid_ = false;
      pid_controller_.reset(io_.position);
    }

    // Write outputs.
    switch (output_type_) {
      case PERCENT:
        // Send the percent output value directly to the motor controller.
        leader_.set(io_.demand);
        break;
      case POSITION:
        // Calculate motor controller voltage from feedback and feedforward.
        double feedback = pid_controller_.calculate(io_.position);
        TrapezoidProfile.State setpoint = pid_controller_.getSetpoint();
        double feedforward = feedforward_.calculate(setpoint.position, setpoint.velocity,
            (setpoint.velocity - last_velocity_setpoint_) / 0.02);

        // Store last velocity setpoint.
        last_velocity_setpoint_ = setpoint.velocity;

        // Set voltage.
        leader_.setVoltage(feedback + feedforward);
        break;
    }
  }

  /**
   * Sets brake mode on the hood.
   *
   * @param value Whether brake mode should be enabled.
   */
  public void setBrakeMode(boolean value) {
    leader_.setIdleMode(value ? IdleMode.kBrake : IdleMode.kCoast);
  }

  /**
   * Sets the % output on the hood.
   *
   * @param value The % output in [-1, 1].
   */
  public void setPercent(double value) {
    reset_pid_ = true;
    output_type_ = OutputType.PERCENT;
    io_.demand = value;
  }

  /**
   * Sets the hood position.
   *
   * @param value The hood position in radians.
   */
  public void setPosition(double value) {
    output_type_ = OutputType.POSITION;
    io_.demand = MathUtil.clamp(value, Constants.kMinAngle + Math.toRadians(0.5), Constants.kMaxAngle);
    pid_controller_.setGoal(io_.demand);
  }

  /**
   * Returns whether the hood is at the position goal.
   *
   * @return Whether the hood is at the position goal.
   */
  public boolean atGoal() {
    return pid_controller_.atGoal();
  }

  /**
   * Returns the current hood position in radians.
   *
   * @return The current hood position in radians.
   */
  public double getPosition() {
    return io_.position;
  }

  public enum OutputType {
    PERCENT, POSITION
  }

  public static class PeriodicIO {
    // Inputs
    double raw_enc;
    double position;
    double supply_current;

    // Outputs
    double demand;
  }

  public static class Constants {
    // Motor Controllers
    public static final int kLeaderId = 8;

    // Sensors
    public static final int kEncoderId = 1;

    // Current Limits
    public static final int kCurrentLimit = 20;

    // Hardware
    public static final double kMinEncoderValue = 0.599;
    public static final double kMaxEncoderValue = 0.837;
    public static final double kMaxAngle = Units.degreesToRadians(42.4);
    public static final double kMinAngle = Units.degreesToRadians(2.6);
    public static final double kEncoderSlope =
        (kMaxAngle - kMinAngle) / (kMaxEncoderValue - kMinEncoderValue);

    // Control
    public static final double kS = 0.00;
    public static final double kG = 0.0;
    public static final double kV = 2.736;
    public static final double kA = 0.03;
    public static final double kP = 5.0;
    public static final double kMaxVelocity = 2 * Math.PI;
    public static final double kMaxAcceleration = 2 * Math.PI;
    public static final double kTolerance = Math.toRadians(3);
  }
}
