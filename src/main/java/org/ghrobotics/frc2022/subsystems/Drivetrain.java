package org.ghrobotics.frc2022.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless;
import static org.ghrobotics.frc2022.subsystems.Drivetrain.Constants.*;

public class Drivetrain extends SubsystemBase {
  // Motor Controllers
  private final CANSparkMax left_leader_ = new CANSparkMax(kLeftLeader, kBrushless);
  private final CANSparkMax left_follower_ = new CANSparkMax(kLeftFollower, kBrushless);
  private final CANSparkMax right_leader_ = new CANSparkMax(kRightLeader, kBrushless);
  private final CANSparkMax right_follower_ = new CANSparkMax(kRightFollower, kBrushless);

  // Encoders
  private final RelativeEncoder left_encoder_;
  private final RelativeEncoder right_encoder_;

  // Gyro
  private final AHRS gyro_ = new AHRS(SPI.Port.kMXP);

  // Feedforward
  private final SimpleMotorFeedforward left_ff_;
  private final SimpleMotorFeedforward right_ff_;

  // Feedback
  private final SparkMaxPIDController left_fb_;
  private final SparkMaxPIDController right_fb_;

  // IO
  private OutputType output_type_ = OutputType.VOLTAGE;
  private final PeriodicIO io_ = new PeriodicIO();

  // Constructor
  public Drivetrain() {
    // Restore factory defaults.
    left_leader_.restoreFactoryDefaults();
    left_follower_.restoreFactoryDefaults();
    right_leader_.restoreFactoryDefaults();
    right_follower_.restoreFactoryDefaults();

    // Invert right side of drivetrain.
    right_leader_.setInverted(true);

    // Set leader-follower relationship.
    left_follower_.follow(left_leader_);
    right_follower_.follow(right_leader_);

    // Initialize encoders.
    left_encoder_ = left_leader_.getEncoder();
    left_encoder_.setPositionConversionFactor(1 / kGearRatio * 2 * Math.PI * kWheelRadius);
    left_encoder_.setVelocityConversionFactor(1 / kGearRatio * 2 * Math.PI * kWheelRadius / 60);
    right_encoder_ = right_leader_.getEncoder();
    right_encoder_.setPositionConversionFactor(1 / kGearRatio * 2 * Math.PI * kWheelRadius);
    right_encoder_.setVelocityConversionFactor(1 / kGearRatio * 2 * Math.PI * kWheelRadius / 60);

    // Reset all sensors.
    reset();

    // Initialize feedforward.
    left_ff_ = new SimpleMotorFeedforward(kLeftKs, kLeftKv, kLeftKa);
    right_ff_ = new SimpleMotorFeedforward(kRightKs, kRightKv, kRightKa);

    // Initialize feedback.
    left_fb_ = left_leader_.getPIDController();
    left_fb_.setP(kLeftKp);
    right_fb_ = right_leader_.getPIDController();
    right_fb_.setP(kRightKp);
  }

  // Runs periodically every 20 ms.
  @Override
  public void periodic() {
    // Read inputs.
    io_.robot_controller_voltage = RobotController.getBatteryVoltage();
    io_.l_position = left_encoder_.getPosition();
    io_.r_position = right_encoder_.getPosition();
    io_.l_velocity = left_encoder_.getVelocity();
    io_.r_velocity = right_encoder_.getVelocity();
    io_.angle = gyro_.getRotation2d();

    SmartDashboard.putNumber("L Position", io_.l_position);
    SmartDashboard.putNumber("R Position", io_.r_position);
    SmartDashboard.putNumber("Angle", io_.angle.getDegrees());


    // Write outputs.
    switch (output_type_) {
      case VOLTAGE:
        left_leader_.setVoltage(io_.l_demand);
        right_leader_.setVoltage(io_.r_demand);
        break;
      case VELOCITY:
        double left_voltage = left_ff_.calculate(io_.l_demand, 0);
        double right_voltage = right_ff_.calculate(io_.r_demand, 0);
        left_fb_.setReference(io_.l_demand, CANSparkMax.ControlType.kVelocity, 0, left_voltage);
        right_fb_.setReference(io_.l_demand, CANSparkMax.ControlType.kVelocity, 0, right_voltage);
        break;
    }
  }

  // Sets the desired motor voltages.
  public void setVoltages(double left, double right) {
    output_type_ = OutputType.VOLTAGE;
    io_.l_demand = left;
    io_.r_demand = right;
  }

  // Sets the desired motor percent outputs. This is automatically multiplied by the robot
  // controller voltage to determine the appropriate voltages.
  public void setPercents(double left, double right) {
    setVoltages(left * io_.robot_controller_voltage, right * io_.robot_controller_voltage);
  }

  public void setVelocity(double left, double right) {
    output_type_ = OutputType.VELOCITY;
    io_.l_demand = left;
    io_.r_demand = right;
  }

  public void reset() {
    left_encoder_.setPosition(0);
    right_encoder_.setPosition(0);
    gyro_.reset();
  }

  enum OutputType {
    VOLTAGE, VELOCITY
  }

  // IO
  static class PeriodicIO {
    // Inputs
    double robot_controller_voltage;
    double l_position;
    double r_position;
    double l_velocity;
    double r_velocity;
    Rotation2d angle;

    // Outputs
    double l_demand;
    double r_demand;
  }

  // Constants
  static class Constants {
    // Motor Controller IDs
    public static final int kLeftLeader = 1;
    public static final int kLeftFollower = 2;
    public static final int kRightLeader = 3;
    public static final int kRightFollower = 4;

    // Hardware
    public static double kGearRatio = 7.29;
    public static double kWheelRadius = 0.076;

    // Feedforward
    public static double kLeftKs = 0.15323;
    public static double kLeftKv = 1.9181;
    public static double kLeftKa = 0.11118;
    public static double kRightKs = 0.17425;
    public static double kRightKv = 1.9669;
    public static double kRightKa = 0.12036;

    // Feedback
    public static double kLeftKp = 3.0;
    public static double kRightKp = 3.0;
  }
}
