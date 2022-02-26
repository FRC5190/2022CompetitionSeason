package org.ghrobotics.frc2022.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.music.Orchestra;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.List;

public class Climber extends SubsystemBase {
  // Motor Controllers
  private final WPI_TalonFX left_leader_;
  private final WPI_TalonFX right_leader_;

  // Pneumatics
  private final Solenoid brake_;
  private final Solenoid left_pivot_;
  private final Solenoid right_pivot_;

  // Sensors
  private final DigitalInput left_rev_limit_switch_;
  private final DigitalInput right_rev_limit_switch_;

  // Control
  private final SimpleMotorFeedforward left_feedforward_;
  private final SimpleMotorFeedforward right_feedforward_;

  // Orchestra
  private final Orchestra orchestra_;

  // IO
  private final PeriodicIO io_ = new PeriodicIO();
  private OutputType output_type_ = OutputType.PERCENT;

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
    left_leader_.setInverted(true);

    right_leader_ = new WPI_TalonFX(Constants.kRightLeaderId);
    right_leader_.configFactoryDefault();
    right_leader_.setNeutralMode(NeutralMode.Brake);
    right_leader_.configVoltageCompSaturation(12);
    right_leader_.enableVoltageCompensation(true);
    right_leader_.setInverted(false);

    // Initialize pneumatics.
    brake_ = new Solenoid(PneumaticsModuleType.REVPH, Constants.kBrakeId);
    left_pivot_ = new Solenoid(PneumaticsModuleType.REVPH, Constants.kLeftPivotId);
    right_pivot_ = new Solenoid(PneumaticsModuleType.REVPH, Constants.kRightPivotId);

    // Initialize limit switches.
    left_rev_limit_switch_ = new DigitalInput(Constants.kLeftRevLimitSwitchId);
    right_rev_limit_switch_ = new DigitalInput(Constants.kRightRevLimitSwitchId);

    // Initialize feedforward.
    left_feedforward_ = new SimpleMotorFeedforward(Constants.kLeftKs, Constants.kLeftKv,
        Constants.kLeftKa);
    right_feedforward_ = new SimpleMotorFeedforward(Constants.kRightKs, Constants.kRightKv,
        Constants.kRightKa);

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
    io_.l_position = left_leader_.getSelectedSensorPosition();
    io_.r_position = right_leader_.getSelectedSensorPosition();
    io_.l_supply_current = left_leader_.getSupplyCurrent();
    io_.r_supply_current = right_leader_.getSupplyCurrent();
    io_.l_rev_limit_switch = left_rev_limit_switch_.get();
    io_.r_rev_limit_switch = right_rev_limit_switch_.get();

    // Write outputs.
    // Update pneumatics if a change is required.
    if (io_.wants_pneumatics_update) {
      io_.wants_pneumatics_update = false;
      brake_.set(io_.brake_value);
      left_pivot_.set(io_.l_pivot_value);
      right_pivot_.set(io_.r_pivot_value);
    }

    // Reset encoder to zero if limit switches are active.
    if (io_.l_rev_limit_switch)
      left_leader_.setSelectedSensorPosition(0);

    if (io_.r_rev_limit_switch)
      right_leader_.setSelectedSensorPosition(0);

    // Set motor outputs.
    switch (output_type_) {
      case PERCENT:
        // Stop the orchestra if it is playing.
        if (orchestra_.isPlaying())
          orchestra_.stop();

        // Send the percent output values directly to the motor controllers. If limit switches
        // are active, clamp output to 0.
        left_leader_.set(Math.max(io_.l_rev_limit_switch ? 0 : -1, io_.l_demand));
        right_leader_.set(Math.max(io_.r_rev_limit_switch ? 0 : -1, io_.r_demand));
        break;
      case POSITION:
        // Stop the orchestra if it is playing.
        if (orchestra_.isPlaying())
          orchestra_.stop();

        // Compute feedforward values and add to built-in motor controller PID.
        left_leader_.set(ControlMode.MotionMagic, Math.max(0, io_.l_demand),
            DemandType.ArbitraryFeedForward,
            left_feedforward_.calculate(left_leader_.getActiveTrajectoryVelocity()));
        right_leader_.set(ControlMode.MotionMagic, Math.max(0, io_.r_demand),
            DemandType.ArbitraryFeedForward,
            right_feedforward_.calculate(right_leader_.getActiveTrajectoryVelocity()));

        break;
      case ORCHESTRA:
        // Play the orchestra.
        orchestra_.play();
    }
  }

  /**
   * Sets the % output on the climber.
   *
   * @param l The left % output in [-1, 1].
   * @param r The right % output in [-1. 1].
   */
  public void setPercent(double l, double r) {
    output_type_ = OutputType.PERCENT;
    io_.l_demand = l;
    io_.r_demand = r;
  }

  /**
   * Sets the position of the climber.
   *
   * @param l The position of the left arm in meters.
   * @param r The position of the right arm in meters.
   */
  public void setPosition(double l, double r) {
    output_type_ = OutputType.POSITION;
    io_.l_position = l;
    io_.r_position = r;
  }

  /**
   * Sets the climber pivot.
   *
   * @param l The left pivot value; true when arm should be pivoted back.
   * @param r The right pivot value; true when arm should be pivoted back.
   */
  public void setPivot(boolean l, boolean r) {
    io_.wants_pneumatics_update = true;
    io_.l_pivot_value = l;
    io_.r_pivot_value = r;
  }

  /**
   * Sets the climber brake.
   *
   * @param value The brake value; true if brake should be engaged.
   */
  public void setBrake(boolean value) {
    io_.wants_pneumatics_update = true;
    io_.brake_value = value;
  }

  /**
   * Plays the orchestra on the Falcon 500 motors powering the climber. The selected song is
   * "Imperial March" by John Williams.
   */
  public void setOrchestra() {
    output_type_ = OutputType.ORCHESTRA;
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

    boolean brake_value;
    boolean l_pivot_value;
    boolean r_pivot_value;
    boolean wants_pneumatics_update;
  }

  public static class Constants {
    // Motor Controllers
    public static final int kLeftLeaderId = 14;
    public static final int kRightLeaderId = 15;

    // Pneumatics
    public static final int kBrakeId = 1;
    public static final int kLeftPivotId = 3;
    public static final int kRightPivotId = 2;

    // Sensors
    public static final int kLeftRevLimitSwitchId = 0;
    public static final int kRightRevLimitSwitchId = 1;

    // Control
    public static final int kRightKs = 0;
    public static final int kLeftKs = 0;
    public static final int kRightKv = 0;
    public static final int kLeftKv = 0;
    public static final int kRightKa = 0;
    public static final int kLeftKa = 0;
  }
}
