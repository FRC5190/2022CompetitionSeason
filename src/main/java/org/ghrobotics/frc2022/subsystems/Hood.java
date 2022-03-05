package org.ghrobotics.frc2022.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.ghrobotics.frc2022.RobotState;
import static com.revrobotics.CANSparkMax.IdleMode;
import static com.revrobotics.CANSparkMax.SoftLimitDirection;
import static com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Hood extends SubsystemBase {
  // Robot State
  private final RobotState robot_state_;

  // Motor Controllers
  private final CANSparkMax leader_;

  // Sensors
  private final RelativeEncoder encoder_;

  // Control
  private final SparkMaxPIDController pid_controller_;

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
    leader_.setIdleMode(IdleMode.kCoast);
    leader_.enableVoltageCompensation(12);
    leader_.setSmartCurrentLimit(Constants.kCurrentLimit);
    leader_.setInverted(true);

    // Initialize encoder.
    encoder_ = leader_.getEncoder();
    encoder_.setPositionConversionFactor(2 * Math.PI / Constants.kGearRatio);
    encoder_.setVelocityConversionFactor(2 * Math.PI / Constants.kGearRatio / 60);

    // Initialize PID controller.
    pid_controller_ = leader_.getPIDController();
    pid_controller_.setP(Constants.kP);
    pid_controller_.setSmartMotionMaxVelocity(Constants.kMaxVelocity, 0);
    pid_controller_.setSmartMotionMaxAccel(Constants.kMaxAcceleration, 0);

    // Set and enable soft limits.
    leader_.setSoftLimit(SoftLimitDirection.kForward, (float) Constants.kMaxAngle);
    leader_.setSoftLimit(SoftLimitDirection.kReverse, (float) Constants.kMinAngle);
    leader_.enableSoftLimit(SoftLimitDirection.kForward, true);
    leader_.enableSoftLimit(SoftLimitDirection.kReverse, true);

    // Assume hood starts at min angle and zero encoder as such.
    encoder_.setPosition(Constants.kMinAngle);
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

    // Update robot state.
    robot_state_.updateHoodAngle(new Rotation2d(io_.position));

    // Write outputs.
    switch (output_type_) {
      case PERCENT:
        // Send the percent output value directly to the motor controller.
        leader_.set(io_.demand);
        break;
      case POSITION:
        // Pass setpoint to built-in motor controller PID.
        pid_controller_.setReference(io_.demand, CANSparkMax.ControlType.kSmartMotion);
        break;
    }
  }

  /**
   * Sets the % output on the hood.
   *
   * @param value The % output in [-1, 1].
   */
  public void setPercent(double value) {
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
    io_.demand = MathUtil.clamp(value, Constants.kMinAngle, Constants.kMaxAngle);
  }

  /**
   * Returns the current hood position in radians.
   *
   * @return The current hood position in radians.
   */
  public double getPosition() {
    return io_.position;
  }

  /**
   * Returns the current hood velocity in radians / sec.
   *
   * @return The current hood velocity in radians / sec.
   */
  public double getVelocity() {
    return io_.velocity;
  }

  public enum OutputType {
    PERCENT, POSITION
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
    public static final int kLeaderId = 8;

    // Current Limits
    public static final int kCurrentLimit = 20;

    // Hardware
    public static final double kGearRatio = 35.0 * 40.0 / 18.0;
    public static final double kMaxAngle = Units.degreesToRadians(55);
    public static final double kMinAngle = Units.degreesToRadians(15);

    // Control
    public static final double kP = 0.05;
    public static final double kMaxVelocity = 2 * Math.PI;
    public static final double kMaxAcceleration = 0.2 * Math.PI;
  }
}

