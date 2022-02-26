package org.ghrobotics.frc2022.subsystems;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.ghrobotics.frc2022.RobotState;
import static com.revrobotics.CANSparkMax.ControlType;
import static com.revrobotics.CANSparkMax.IdleMode;
import static com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Drivetrain extends SubsystemBase {
  // Robot State
  private final RobotState robot_state_;

  // Motor Controllers
  private final CANSparkMax left_leader_;
  private final CANSparkMax left_follower_;
  private final CANSparkMax right_leader_;
  private final CANSparkMax right_follower_;

  // Sensors
  private final RelativeEncoder left_encoder_;
  private final RelativeEncoder right_encoder_;
  private final WPI_PigeonIMU gyro_;

  // Control
  private final SparkMaxPIDController left_pid_controller_;
  private final SparkMaxPIDController right_pid_controller_;
  private final SimpleMotorFeedforward left_feedforward_;
  private final SimpleMotorFeedforward right_feedforward_;

  // Trajectory Tracking
  private final DifferentialDriveKinematics kinematics_;
  private final RamseteController ramsete_controller_;


  // IO
  private OutputType output_type_ = OutputType.PERCENT;
  private final PeriodicIO io_ = new PeriodicIO();

  /**
   * Constructs an instance of the Drivetrain subsystem. Only one instance of this subsystem should
   * be created in the main Robot class and references to this instance should be passed around
   * the robot code.
   *
   * @param robot_state Reference to the global robot state instance.
   */
  public Drivetrain(RobotState robot_state) {
    // Store reference to robot state.
    robot_state_ = robot_state;

    // Initialize motor controllers.
    left_leader_ = new CANSparkMax(Constants.kLeftLeaderId, MotorType.kBrushless);
    left_leader_.restoreFactoryDefaults();
    left_leader_.setIdleMode(IdleMode.kBrake);
    left_leader_.enableVoltageCompensation(12);
    left_leader_.setInverted(false);

    right_leader_ = new CANSparkMax(Constants.kRightLeaderId, MotorType.kBrushless);
    right_leader_.restoreFactoryDefaults();
    right_leader_.setIdleMode(IdleMode.kBrake);
    right_leader_.setInverted(true);

    left_follower_ = new CANSparkMax(Constants.kLeftFollowerId, MotorType.kBrushless);
    left_follower_.restoreFactoryDefaults();
    left_follower_.setIdleMode(IdleMode.kBrake);
    left_follower_.enableVoltageCompensation(12);
    left_follower_.follow(left_leader_);

    right_follower_ = new CANSparkMax(Constants.kRightFollowerId, MotorType.kBrushless);
    right_follower_.restoreFactoryDefaults();
    right_follower_.setIdleMode(IdleMode.kBrake);
    right_follower_.enableVoltageCompensation(12);
    right_follower_.follow(right_leader_);

    // Initialize encoders.
    left_encoder_ = left_leader_.getEncoder();
    left_encoder_.setPositionConversionFactor(
        2 * Math.PI * Constants.kWheelRadius / Constants.kGearRatio);
    left_encoder_.setVelocityConversionFactor(
        2 * Math.PI * Constants.kWheelRadius / Constants.kGearRatio / 60);

    right_encoder_ = right_leader_.getEncoder();
    right_encoder_.setPositionConversionFactor(
        2 * Math.PI * Constants.kWheelRadius / Constants.kGearRatio);
    right_encoder_.setVelocityConversionFactor(
        2 * Math.PI * Constants.kWheelRadius / Constants.kGearRatio / 60);

    // Initialize gyro.
    gyro_ = new WPI_PigeonIMU(Constants.kPigeonIMUId);

    // Initialize PID controllers.
    left_pid_controller_ = left_leader_.getPIDController();
    left_pid_controller_.setP(Constants.kLeftKp);

    right_pid_controller_ = right_leader_.getPIDController();
    right_pid_controller_.setP(Constants.kRightKp);

    // Initialize feedforward.
    left_feedforward_ = new SimpleMotorFeedforward(
        Constants.kLeftKs, Constants.kLeftKv, Constants.kLeftKa);
    right_feedforward_ = new SimpleMotorFeedforward(Constants.kRightKs, Constants.kRightKv,
        Constants.kRightKa);

    // Initialize trajectory tracking.
    kinematics_ = new DifferentialDriveKinematics(Constants.kTrackWidth);
    ramsete_controller_ = new RamseteController();
  }

  /**
   * This method runs periodically every 20 ms. Here, all sensor values are read and all motor
   * outputs should be set.
   */
  @Override
  public void periodic() {
    // Read inputs.
    io_.l_position = left_encoder_.getPosition();
    io_.r_position = right_encoder_.getPosition();
    io_.l_velocity = left_encoder_.getVelocity();
    io_.r_velocity = right_encoder_.getVelocity();
    io_.angle = gyro_.getRotation2d();
    io_.angular_rate = -Math.toRadians(gyro_.getRate());

    // Update robot state with measurements.
    robot_state_.updateRobotPose(
        io_.l_position, io_.r_position, io_.l_velocity, io_.r_velocity, io_.angle);
    robot_state_.updateRobotSpeeds(
        new ChassisSpeeds((io_.l_velocity + io_.r_velocity) / 2, 0, io_.angular_rate));

    // Write outputs.
    switch (output_type_) {
      case PERCENT:
        // Send the percent output values directly to the motor controller.
        left_leader_.set(io_.l_demand);
        right_leader_.set(io_.r_demand);
        break;
      case VELOCITY:
        // Calculate feedforward value and add to built-in motor controller PID.
        left_pid_controller_.setReference(io_.l_demand, ControlType.kVelocity, 0,
            left_feedforward_.calculate(io_.l_demand, (io_.l_demand - io_.l_velocity) / 0.02));
        right_pid_controller_.setReference(io_.r_demand, ControlType.kVelocity, 0,
            right_feedforward_.calculate(io_.r_demand, (io_.r_demand - io_.r_velocity) / 0.02));
        break;
    }
  }

  /**
   * Sets the idle mode on each of the drivetrain motors.
   *
   * @param mode The desired idle mode (brake or coast).
   */
  public void setIdleMode(IdleMode mode) {
    left_leader_.setIdleMode(mode);
    left_follower_.setIdleMode(mode);
    right_leader_.setIdleMode(mode);
    right_follower_.setIdleMode(mode);
  }

  /**
   * Sets the % output on the drivetrain.
   *
   * @param l The left % output in [-1, 1].
   * @param r The right % output in [-1, 1].
   */
  public void setPercent(double l, double r) {
    output_type_ = OutputType.PERCENT;
    io_.l_demand = l;
    io_.r_demand = r;
  }

  /**
   * Sets the desired drivetrain velocity for closed loop control.
   *
   * @param l Desired left velocity in m/s.
   * @param r Desired right velocity in m/s.
   */
  public void setVelocity(double l, double r) {
    output_type_ = OutputType.VELOCITY;
    io_.l_demand = l;
    io_.r_demand = r;
  }

  /**
   * Returns the left position in meters.
   *
   * @return The left position in meters.
   */
  public double getLeftPosition() {
    return io_.l_position;
  }

  /**
   * Returns the right position in meters.
   *
   * @return The right position in meters.
   */
  public double getRightPosition() {
    return io_.r_position;
  }

  /**
   * Returns the left velocity in meters per second.
   *
   * @return The left velocity in meters per second.
   */
  public double getLeftVelocity() {
    return io_.l_velocity;
  }

  /**
   * Returns the right velocity in meters per second.
   *
   * @return The right velocity in meters per second.
   */
  public double getRightVelocity() {
    return io_.r_velocity;
  }

  /**
   * Returns the kinematics for the drivetrain.
   *
   * @return The kinematics for the drivetrain.
   */
  public DifferentialDriveKinematics getKinematics() {
    return kinematics_;
  }

  /**
   * Returns the Ramsete controller for this drivetrain.
   *
   * @return The Ramsete controller for this drivetrain.
   */
  public RamseteController getRamseteController() {
    return ramsete_controller_;
  }

  enum OutputType {
    PERCENT, VELOCITY
  }

  public static class PeriodicIO {
    // Inputs
    double l_position;
    double r_position;
    double l_velocity;
    double r_velocity;
    Rotation2d angle;
    double angular_rate;

    // Outputs
    double l_demand;
    double r_demand;
  }

  public static class Constants {
    // Motor Controller IDs
    public static final int kLeftLeaderId = 1;
    public static final int kLeftFollowerId = 2;
    public static final int kRightLeaderId = 3;
    public static final int kRightFollowerId = 4;

    // Sensors
    public static final int kPigeonIMUId = 17;

    // Hardware
    public static double kGearRatio = 6.07;
    public static double kWheelRadius = 0.0508;
    public static double kTrackWidth = 0.759;

    // Control
    public static double kLeftKs = 0;
    public static double kLeftKv = 0;
    public static double kLeftKa = 0;
    public static double kLeftKp = 0;
    public static double kRightKs = 0;
    public static double kRightKv = 0;
    public static double kRightKa = 0;
    public static double kRightKp = 0;
  }
}