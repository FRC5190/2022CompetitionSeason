package org.ghrobotics.frc2022.subsystems;

import com.ctre.phoenix.sensors.BasePigeonSimCollection;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
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
  private double last_l_velocity_setpoint_ = 0;
  private double last_r_velocity_setpoint_ = 0;

  // Trajectory Tracking
  private final DifferentialDriveKinematics kinematics_;
  private final RamseteController ramsete_controller_;

  // Simulation
  private final DifferentialDrivetrainSim physics_sim_;
  private final SimDeviceSim left_leader_sim_;
  private final SimDeviceSim right_leader_sim_;
  private final BasePigeonSimCollection gyro_sim_;

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
    right_leader_.enableVoltageCompensation(12);
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

    // Initialize simulation objects.
    physics_sim_ = new DifferentialDrivetrainSim(
        DCMotor.getNEO(2),
        Constants.kGearRatio,
        Constants.kMOI,
        Constants.kMass,
        Constants.kWheelRadius,
        Constants.kTrackWidth,
        null);
    left_leader_sim_ = new SimDeviceSim("SPARK MAX [" + Constants.kLeftLeaderId + "]");
    right_leader_sim_ = new SimDeviceSim("SPARK MAX [" + Constants.kRightLeaderId + "]");
    gyro_sim_ = gyro_.getSimCollection();
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

    io_.l_leader_supply_current = left_leader_.getOutputCurrent();
    io_.l_follower_supply_current = left_follower_.getOutputCurrent();
    io_.r_leader_supply_current = right_leader_.getOutputCurrent();
    io_.r_follower_supply_current = right_follower_.getOutputCurrent();

    // Update robot state with measurements.
    double avg_velocity = (io_.l_velocity + io_.r_velocity) / 2;
    robot_state_.updateRobotPose(
        io_.l_position, io_.r_position, avg_velocity, io_.angle);
    robot_state_.updateRobotSpeeds(new ChassisSpeeds(avg_velocity, 0, io_.angular_rate));

    // Write outputs.
    switch (output_type_) {
      case PERCENT:
        // Send the percent output values directly to the motor controller.
        left_leader_.set(io_.l_demand);
        right_leader_.set(io_.r_demand);

        // Set simulated inputs.
        if (RobotBase.isSimulation()) {
          left_leader_sim_.getDouble("Applied Output").set(io_.l_demand * 12);
          right_leader_sim_.getDouble("Applied Output").set(io_.r_demand * 12);
        }
        break;
      case VELOCITY:
        // Calculate feedforward value and add to built-in motor controller PID.
        double l_feedforward = left_feedforward_.calculate(io_.l_demand,
            (io_.l_demand - last_l_velocity_setpoint_) / 0.02);
        double r_feedforward = right_feedforward_.calculate(io_.r_demand,
            (io_.r_demand - last_r_velocity_setpoint_) / 0.02);

        left_pid_controller_.setReference(io_.l_demand, ControlType.kVelocity, 0, l_feedforward);
        right_pid_controller_.setReference(io_.r_demand, ControlType.kVelocity, 0, r_feedforward);

        // Set simulated inputs.
        if (RobotBase.isSimulation()) {
          left_leader_sim_.getDouble("Applied Output").set(l_feedforward);
          right_leader_sim_.getDouble("Applied Output").set(r_feedforward);
        }

        // Store last velocity setpoints.
        last_l_velocity_setpoint_ = io_.l_demand;
        last_r_velocity_setpoint_ = io_.r_demand;
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
    physics_sim_.setInputs(left_leader_.getAppliedOutput(), right_leader_.getAppliedOutput());

    // Update physics sim forward in time.
    physics_sim_.update(0.02);

    // Update encoder values.
    left_leader_sim_.getDouble("Position").set(physics_sim_.getLeftPositionMeters());
    left_leader_sim_.getDouble("Velocity").set(physics_sim_.getLeftVelocityMetersPerSecond());
    right_leader_sim_.getDouble("Position").set(physics_sim_.getRightPositionMeters());
    right_leader_sim_.getDouble("Velocity").set(physics_sim_.getRightVelocityMetersPerSecond());

    // Update gyro values.
    gyro_sim_.setRawHeading(physics_sim_.getHeading().getDegrees());
  }


  /**
   * Sets brake mode on each of the drivetrain motors.
   *
   * @param value Whether brake mode should be enabled.
   */
  public void setBrakeMode(boolean value) {
    IdleMode mode = value ? IdleMode.kBrake : IdleMode.kCoast;
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
    last_l_velocity_setpoint_ = 0;
    last_r_velocity_setpoint_ = 0;
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
    Rotation2d angle = new Rotation2d();
    double angular_rate;

    double l_leader_supply_current;
    double l_follower_supply_current;
    double r_leader_supply_current;
    double r_follower_supply_current;

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
    public static double kMass = 65.0;
    public static double kMOI = 10.0;

    // Control
    public static double kLeftKs = 0.25438;
    public static double kLeftKv = 2.404;
    public static double kLeftKa = 0.17931;
    public static double kLeftKp = 0.00010892;
    public static double kRightKs = 0.23742;
    public static double kRightKv = 2.3732;
    public static double kRightKa = 0.14528;
    public static double kRightKp = 0.00010892;
  }
}
