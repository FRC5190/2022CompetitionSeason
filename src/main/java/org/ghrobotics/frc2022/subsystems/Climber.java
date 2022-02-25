package org.ghrobotics.frc2022.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.music.Orchestra;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.List;

public class Climber extends SubsystemBase {
  // Motor Controllers
  private final TalonFX left_leader_;
  private final TalonFX right_leader_;

  // TODO: create pneumatics, control, feedforward, etc.

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
    left_leader_ = new TalonFX(Constants.kLeftLeaderId);
    right_leader_ = new TalonFX(Constants.kRightLeaderId);

    // TODO: initialize pneumatics, control, feedforward, etc.

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
    // TODO: read inputs.

    // Write outputs.
    if (io_.wants_pneumatics_update) {
      io_.wants_pneumatics_update = false;
      // TODO: set solenoids.
    }

    switch (output_type_) {
      case PERCENT:
        // Stop the orchestra if it is playing.
        if (orchestra_.isPlaying())
          orchestra_.stop();

        // TODO: set percent output
        break;
      case POSITION:
        // Stop the orchestra if it is playing.
        if (orchestra_.isPlaying())
          orchestra_.stop();

        // TODO: set position output
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
    // TODO
  }

  /**
   * Sets the position of the climber.
   *
   * @param l The position of the left arm in meters.
   * @param r The position of the right arm in meters.
   */
  public void setPosition(double l, double r) {
    // TODO
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
    // TODO
    return 0;
  }

  /**
   * Returns the right position of the climber in meters.
   *
   * @return The right position of the climber in meters.
   */
  public double getRightPosition() {
    // TODO
    return 0;
  }

  /**
   * Returns the left supply current.
   *
   * @return The left supply current.
   */
  public double getLeftSupplyCurrent() {
    // TODO
    return 0;
  }

  /**
   * Returns the right supply current.
   *
   * @return The right supply current.
   */
  public double getRightSupplyCurrent() {
    // TODO
    return 0;
  }

  /**
   * Returns whether the left reverse limit switch is closed.
   *
   * @return Whether the left reverse limit switch is closed.
   */
  public boolean getLeftReverseLimitSwitchClosed() {
    // TODO
    return false;
  }

  /**
   * Returns whether the right reverse limit switch is closed.
   *
   * @return Whether the right reverse limit switch is closed.
   */
  public boolean getRightReverseLimitSwitchClosed() {
    // TODO
    return false;
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
    public static final int kBrakeId = 0;
    public static final int kLeftPivotId = 1;
    public static final int kRightPivotId = 2;
  }
}
