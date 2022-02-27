package org.ghrobotics.frc2022.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.BooleanSupplier;
import org.ghrobotics.frc2022.subsystems.Climber;

public class ClimbCommand extends CommandBase {
  // References to subsystem, Xbox controller, and climb mdoe..
  private final Climber climber_;
  private final XboxController controller_;
  private final BooleanSupplier climb_mode_;

  // The state of the automated climb.
  private State state_;

  // The getter for advancing the state of the climber.
  private BooleanSupplier advance_state_;

  // Whether we are currently waiting to advance the state.
  private boolean waiting_for_advance_state_;

  public ClimbCommand(Climber climber, XboxController controller, BooleanSupplier climb_mode) {
    // Assign member variables.
    climber_ = climber;
    controller_ = controller;
    climb_mode_ = climb_mode;

    // Set advance state supplier.
    advance_state_ = () -> controller.getRawButton(
        Constants.kAdvanceButton) && climb_mode.getAsBoolean();

    // Set mutable variable default values.
    resetClimbState();

    // Set subsystem requirements.
    addRequirements(climber_);
  }

  @Override
  public void execute() {
    // Manual Mode:
    // TODO

    // Automatic Mode:
    switch (state_) {
      // Climber is stowed and out of the way.
      case STOW:
        // Set both positions to zero and pivot out.
        climber_.setLeftPosition(0);
        climber_.setRightPosition(0);
        climber_.setPivot(true, true);

        // Set brake if the right climber is all the way down.
        if (climber_.getRightReverseLimitSwitchClosed()) {
          climber_.setBrake(true);
        }

        waiting_for_advance_state_ = true;

        // If we are given the directive to advance, change state to "Ready for L2".
        if (advance_state_.getAsBoolean()) {
          waiting_for_advance_state_ = false;
          state_ = State.READY_FOR_L2;
        }
        break;

      // Climber is ready to climb to the middle (L2) rung.
      case READY_FOR_L2:
        // The left arm needs to pivot out and extend all the way to be ready to grab L3. There
        // is no harm in doing that now.
        climber_.setPivot(true, false);
        climber_.setLeftPosition(Climber.Constants.kMaxHeight);

        // The right arm needs to go up 7 inches to be above the L2 bar. Make sure the brake is
        // unlocked, or we will damage the motor.
        double ready_for_l2_right_arm_setpoint = Units.inchesToMeters(7);
        climber_.setBrake(false);
        climber_.setRightPosition(ready_for_l2_right_arm_setpoint);

        // Check whether the right arm has reached the setpoint. If so, we are ready to go to the
        // next step (provided the driver is also ready).
        if (Math.abs(climber_.getRightPosition() - ready_for_l2_right_arm_setpoint) <
            Climber.Constants.kErrorTolerance) {
          waiting_for_advance_state_ = true;
          if (advance_state_.getAsBoolean()) {
            state_ = State.CLIMB_L2;
            waiting_for_advance_state_ = false;
          }
        }
        break;

      // Climber is actively climbing L2.
      case CLIMB_L2:
        // Check whether the left arm is at the setpoint (max height).
        boolean left_arm_ready_for_l3 =
            Math.abs(climber_.getLeftPosition() - Climber.Constants.kMaxHeight) <
                Climber.Constants.kErrorTolerance;

        // If the left arm is at the setpoint, pivot it back in.
        if (left_arm_ready_for_l3) {
          climber_.setPivot(false, false);
        }

        // Bring right arm all the way down until it hits the limit switch. When we hit the limit
        // switch, zero output and set brake.
        if (!climber_.getRightReverseLimitSwitchClosed()) {
          // Pull down with right arm.
          climber_.setRightPercent(-0.5);
        } else {
          // Zero right arm output and set brake.
          climber_.setRightPercent(0);
          climber_.setBrake(true);

          // If the left arm is at setpoint and driver is ready, move on to the next step.
          if (left_arm_ready_for_l3) {
            waiting_for_advance_state_ = true;
            if (advance_state_.getAsBoolean()) {
              state_ = State.CLIMB_L3;
              waiting_for_advance_state_ = false;
            }
          }
        }
        break;

      // Climber is actively climbing L3.
      case CLIMB_L3:
        // Once the left arm has cleared 4 inches from the top, pivot the right arm out and
        // extend to max height.
        if (climber_.getLeftPosition() < Climber.Constants.kMaxHeight - Units.inchesToMeters(4)) {
          climber_.setPivot(false, true);
          climber_.setBrake(false);
          climber_.setRightPosition(Climber.Constants.kMaxHeight);
        }

        // Check whether the right arm is at the setpoint (max height).
        boolean right_arm_ready_for_l4 =
            Math.abs(climber_.getRightPosition() - Climber.Constants.kMaxHeight) <
                Climber.Constants.kErrorTolerance;

        // If the right arm is at the setpoint, pivot it back in.
        if (right_arm_ready_for_l4) {
          climber_.setPivot(false, false);
        }

        // Bring left arm all the way down until it hits the limit switch. When we hit the limit
        // switch, hold position with PID.
        if (!climber_.getLeftReverseLimitSwitchClosed()) {
          // Pull down with left arm.
          climber_.setLeftPercent(-0.5);
        } else {
          // Hold left arm in place.
          climber_.setLeftPosition(0);

          // If the right arm is at setpoint and driver is ready, move on to the next step.
          if (right_arm_ready_for_l4) {
            waiting_for_advance_state_ = true;
            if (advance_state_.getAsBoolean()) {
              state_ = State.CLIMB_L4;
              waiting_for_advance_state_ = false;
            }
          }
          if (right_arm_ready_for_l4 && advance_state_.getAsBoolean()) {
            state_ = State.CLIMB_L4;
          }
        }

        break;

      // Climber is actively climbing L4.
      case CLIMB_L4:
        // Bring right arm all the way down until it hits the limit switch. When we hit the limit
        // switch, zero output and set brake.
        if (!climber_.getRightReverseLimitSwitchClosed()) {
          // Pull down with right arm.
          climber_.setRightPercent(-0.5);
        } else {
          // Zero right arm output and set brake.
          climber_.setRightPercent(0);
          climber_.setBrake(true);
        }
        break;
    }
  }

  /**
   * Resets the auto-climb state.
   */
  public void resetClimbState() {
    waiting_for_advance_state_ = false;
    state_ = State.STOW;
  }

  /**
   * Returns whether we are currently waiting for the state to advance.
   *
   * @return Whether we are currently waiting for the state to advance.
   */
  public boolean isWaitingForStateAdvance() {
    return waiting_for_advance_state_;
  }

  public enum State {
    STOW,
    READY_FOR_L2,
    CLIMB_L2,
    CLIMB_L3,
    CLIMB_L4
  }

  public static class Constants {
    // Buttons
    public static final int kAdvanceButton = XboxController.Button.kStart.value;
  }
}
