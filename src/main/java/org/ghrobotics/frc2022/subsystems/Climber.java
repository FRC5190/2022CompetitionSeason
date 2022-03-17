package org.ghrobotics.frc2022.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.music.Orchestra;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.List;

public class Climber extends SubsystemBase {
  // Motor Controllers
  private final WPI_TalonFX left_leader_;
  private final WPI_TalonFX right_leader_;

  // Pneumatics
  private final Solenoid brake_;
  private final DoubleSolenoid left_pivot_;
  private final DoubleSolenoid right_pivot_;

  // Control
  private final SimpleMotorFeedforward left_feedforward_;
  private final SimpleMotorFeedforward right_feedforward_;

  // Orchestra
  private final Orchestra orchestra_;

  // IO
  private final PeriodicIO io_ = new PeriodicIO();
  private OutputType left_output_type_ = OutputType.PERCENT;
  private OutputType right_output_type_ = OutputType.PERCENT;

  /**
   * Constructs an instance of the Climber subsystem. Only one instance of this subsystem should
   * be created in the main Robot class and references to this instance should be passed around
   * the robot code.
   */
  public Climber() {
    // Initialize motor controllers.
    left_leader_ = new WPI_TalonFX(Constants.kLeftLeaderId);
    left_leader_.configFactoryDefault();
    left_leader_.setNeutralMode(NeutralMode.Brake);
    left_leader_.configVoltageCompSaturation(12);
    left_leader_.enableVoltageCompensation(true);
    left_leader_.setInverted(false);

    right_leader_ = new WPI_TalonFX(Constants.kRightLeaderId);
    right_leader_.configFactoryDefault();
    right_leader_.setNeutralMode(NeutralMode.Brake);
    right_leader_.configVoltageCompSaturation(12);
    right_leader_.enableVoltageCompensation(true);
    right_leader_.setInverted(true);

    // Initialize pneumatics.
    brake_ = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.kBrakeId);
    left_pivot_ = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.kLeftPivotForwardId, 
        Constants.kLeftPivotReverseId);
    right_pivot_ = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.kRightPivotForwardId, 
        Constants.kRightPivotReverseId);

    // Initialize feedback.
    left_leader_.configMotionCruiseVelocity(toCTREVelocity(Constants.kMaxVelocity));
    left_leader_.configMotionAcceleration(toCTREVelocity(Constants.kMaxAcceleration));
    left_leader_.config_kP(0, Constants.kLeftKp);

    right_leader_.configMotionCruiseVelocity(toCTREVelocity(Constants.kMaxVelocity));
    right_leader_.configMotionAcceleration(toCTREVelocity(Constants.kMaxAcceleration));
    right_leader_.config_kP(0, Constants.kRightKp);

    // Initialize feedforward.
    left_feedforward_ = new SimpleMotorFeedforward(Constants.kLeftKs, Constants.kLeftKv,
        Constants.kLeftKa);
    right_feedforward_ = new SimpleMotorFeedforward(Constants.kRightKs, Constants.kRightKv,
        Constants.kRightKa);

    // Configure soft limits.
    left_leader_.configReverseSoftLimitThreshold(0);
    left_leader_.configForwardSoftLimitThreshold(Constants.kMaxHeightNativeUnits);
    right_leader_.configReverseSoftLimitThreshold(0);
    right_leader_.configForwardSoftLimitThreshold(Constants.kMaxHeightNativeUnits);
    enableSoftLimits(true);

    // Initialize orchestra to play music.
    orchestra_ = new Orchestra(List.of(left_leader_, right_leader_));
    orchestra_.loadMusic("imperial_march.chrp");
  }

  /**
   * This method runs periodically every 20 ms. Here, all sensor values are read and all motor
   * and pneumatics outputs should be set.
   */
  @Override
  public void periodic() {
    // Read inputs.
    io_.l_position = fromCTREPosition(left_leader_.getSelectedSensorPosition());
    io_.r_position = fromCTREPosition(right_leader_.getSelectedSensorPosition());
    io_.l_supply_current = left_leader_.getSupplyCurrent();
    io_.r_supply_current = right_leader_.getSupplyCurrent();

    // Zero
    if (io_.wants_zero) {
      io_.wants_zero = false;
      left_leader_.setSelectedSensorPosition(0);
      right_leader_.setSelectedSensorPosition(0);
    }

    // Write outputs.
    // Update pneumatics if a change is required.
    if (io_.wants_pneumatics_update) {
      // Change is no longer required.
      io_.wants_pneumatics_update = false;

      // Set solenoid values.
      brake_.set(!io_.brake_value); // brake is wired backward
      left_pivot_.set(io_.l_pivot_value ? DoubleSolenoid.Value.kForward :  DoubleSolenoid.Value.kReverse);
      right_pivot_.set(io_.r_pivot_value ? DoubleSolenoid.Value.kForward :  DoubleSolenoid.Value.kReverse);
    }

    // Set motor outputs.
    switch (left_output_type_) {
      case PERCENT:
        // Send the percent output values directly to the motor controller.
        left_leader_.set(ControlMode.PercentOutput, io_.l_demand);
        break;
      case POSITION:
        // Calculate feedforward value and add to built-in motor controller PID.
        left_leader_.set(ControlMode.MotionMagic, io_.l_demand, DemandType.ArbitraryFeedForward,
            left_feedforward_.calculate(left_leader_.getActiveTrajectoryVelocity()));
        break;
    }

    // If brake is engaged, don't do anything to prevent damage.
    if (io_.brake_value && right_output_type_ != OutputType.ORCHESTRA) {
      // Set output to zero and exit out of here.
      right_leader_.set(0);
      return;
    }

    switch (right_output_type_) {
      case PERCENT:
        // Send the percent output values directly to the motor controller.
        right_leader_.set(io_.r_demand);
        break;
      case POSITION:
        // Calculate feedforward value and add to built-in motor controller PID.
        right_leader_.set(ControlMode.MotionMagic, io_.r_demand, DemandType.ArbitraryFeedForward,
            right_feedforward_.calculate(right_leader_.getActiveTrajectoryVelocity()));
        break;
    }

    // If we want orchestra, play that.
    if (left_output_type_ == OutputType.ORCHESTRA && right_output_type_ == OutputType.ORCHESTRA)
      orchestra_.play();
  }

  public void zero() {
    io_.wants_zero = true;
  }

  /**
   * Enables or disables soft limits for each motor controller. This needs to be done to zero the
   * climber at the start of the match.
   *
   * @param value Whether the soft limits should be enforced.
   */
  public void enableSoftLimits(boolean value) {
    left_leader_.configForwardSoftLimitEnable(value);
    left_leader_.configReverseSoftLimitEnable(value);
    right_leader_.configForwardSoftLimitEnable(value);
    right_leader_.configReverseSoftLimitEnable(value);
  }

  /**
   * Sets the left arm % output.
   *
   * @param value The left arm % output in [-1, 1].
   */
  public void setLeftPercent(double value) {
    left_output_type_ = OutputType.PERCENT;
    io_.l_demand = value;
  }

  /**
   * Sets the right arm % output.
   *
   * @param value The right arm % output in [-1, 1].
   */
  public void setRightPercent(double value) {
    right_output_type_ = OutputType.PERCENT;
    io_.r_demand = value;
  }

  /**
   * Sets the left arm position.
   *
   * @param value The left arm position in meters.
   */
  public void setLeftPosition(double value) {
    left_output_type_ = OutputType.POSITION;
    io_.l_demand = toCTREPosition(value);
  }

  /**
   * Sets the right arm position.
   *
   * @param value The right arm position in meters.
   */
  public void setRightPosition(double value) {
    right_output_type_ = OutputType.POSITION;
    io_.r_demand = toCTREPosition(value);
  }

  /**
   * Sets the climber pivot.
   *
   * @param l The left pivot value; true when arm should be pivoted back.
   * @param r The right pivot value; true when arm should be pivoted back.
   */
  public void setPivot(boolean l, boolean r) {
    io_.wants_pneumatics_update = l != io_.l_pivot_value || r != io_.r_pivot_value;
    io_.l_pivot_value = l;
    io_.r_pivot_value = r;
  }

  /**
   * Sets the climber brake.
   *
   * @param value The brake value; true if brake should be engaged.
   */
  public void setBrake(boolean value) {
    io_.wants_pneumatics_update = value != io_.brake_value;
    io_.brake_value = value;
  }

  /**
   * Plays the orchestra on the Falcon 500 motors powering the climber. The selected song is
   * "Imperial March" by John Williams.
   */
  public void setOrchestra() {
    left_output_type_ = OutputType.ORCHESTRA;
    right_output_type_ = OutputType.ORCHESTRA;
  }

  /**
   * Returns the left position of the climber in meters.
   *
   * @return The left position of the climber in meters.
   */
  public double getLeftPosition() {
    return io_.l_position;
  }

  /**
   * Returns the right position of the climber in meters.
   *
   * @return The right position of the climber in meters.
   */
  public double getRightPosition() {
    return io_.r_position;
  }

  /**
   * Returns the left supply current.
   *
   * @return The left supply current.
   */
  public double getLeftSupplyCurrent() {
    return io_.l_supply_current;
  }

  /**
   * Returns the right supply current.
   *
   * @return The right supply current.
   */
  public double getRightSupplyCurrent() {
    return io_.r_supply_current;
  }

  /**
   * Returns whether the left reverse limit switch is closed.
   *
   * @return Whether the left reverse limit switch is closed.
   */
  public boolean getLeftReverseLimitSwitchClosed() {
    return io_.l_rev_limit_switch;
  }

  /**
   * Returns whether the right reverse limit switch is closed.
   *
   * @return Whether the right reverse limit switch is closed.
   */
  public boolean getRightReverseLimitSwitchClosed() {
    return io_.r_rev_limit_switch;
  }

  /**
   * Returns the state of the left pivot.
   *
   * @return The state of the left pivot.
   */
  public boolean getLeftPivot() {
    return io_.l_pivot_value;
  }

  /**
   * Returns the state of the right pivot.
   *
   * @return The state of the brake.
   */
  public boolean getRightPivot() {
    return io_.r_pivot_value;
  }

  /**
   * Returns the state of the brake.
   *
   * @return The state of the brake.
   */
  public boolean getBrake() {
    return io_.brake_value;
  }

  /**
   * Converts position to CTRE native units.
   *
   * @param pos The position in meters.
   * @return The position in native units.
   */
  private static double toCTREPosition(double pos) {
    return pos * Constants.kMaxHeightNativeUnits / Constants.kMaxHeight;
  }

  /**
   * Converts position to CTRE native units / 100 ms.
   *
   * @param vel The velocity in meters per second.
   * @return The velocity in native units / 100 ms.
   */
  private static double toCTREVelocity(double vel) {
    return toCTREPosition(vel) * 10;
  }

  /**
   * Converts CTRE native units to position.
   *
   * @param pos The position in native units.
   * @return The position in meters.
   */
  private static double fromCTREPosition(double pos) {
    return pos * Constants.kMaxHeight / Constants.kMaxHeightNativeUnits;
  }

  /**
   * Converts CTRE native units / 100 ms to velocity.
   *
   * @param vel The velocity in native units / 100 ms.
   * @return The velocity in meters per second.
   */
  private static double fromCTREVelocity(double vel) {
    return fromCTREPosition(vel) / 10;
  }

  public enum OutputType {
    PERCENT, POSITION, ORCHESTRA
  }

  public static class PeriodicIO {
    // Inputs
    double l_position;
    double r_position;
    double l_supply_current;
    double r_supply_current;
    boolean l_rev_limit_switch;
    boolean r_rev_limit_switch;

    // Outputs
    double l_demand;
    double r_demand;

    boolean wants_zero = false;

    boolean brake_value = true;
    boolean l_pivot_value;
    boolean r_pivot_value;
    boolean wants_pneumatics_update;
  }

  public static class Constants {
    // Motor Controllers
    public static final int kLeftLeaderId = 14;
    public static final int kRightLeaderId = 15;

    // Pneumatics
    public static final int kBrakeId = 0;
    public static final int kLeftPivotForwardId = 1;
    public static final int kLeftPivotReverseId = 2;
    public static final int kRightPivotForwardId = 3;
    public static final int kRightPivotReverseId = 4;

    // Sensors
    public static final int kLeftRevLimitSwitchId = 8;
    public static final int kRightRevLimitSwitchId = 9;

    // Hardware
    public static final double kMaxHeightNativeUnits = 153000;
    public static final double kMaxHeight = Units.inchesToMeters(25.75);

    // Control
    public static final double kLeftKs = 0;
    public static final double kLeftKv = 0;
    public static final double kLeftKa = 0;
    public static final double kLeftKp = 0.2;

    public static final double kRightKs = 0;
    public static final double kRightKv = 0;
    public static final double kRightKa = 0;
    public static final double kRightKp = 0.2;

    public static final double kMaxVelocity = Units.inchesToMeters(0.1);
    public static final double kMaxAcceleration = Units.inchesToMeters(0.1);
    public static final double kErrorTolerance = Units.inchesToMeters(1.0);
  }
}
