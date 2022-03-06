package org.ghrobotics.frc2022.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.BooleanSupplier;
import org.ghrobotics.frc2022.subsystems.Climber;

public class ClimbTeleop extends CommandBase {
  // Reference to subsystem, controller, and climb mode supplier.
  private final Climber climber_;
  private final XboxController controller_;
  private final BooleanSupplier climb_mode_;

  // Store arm setpoint for closed loop control.
  private boolean left_arm_setpoint_set_;
  private boolean right_arm_setpoint_set_;

  /**
   * Controls the climber with manual inputs from the Xbox controller.
   *
   * @param climber    Reference to climber subsystem.
   * @param controller Reference to Xbox controller used to control the subsystem.
   * @param climb_mode Whether we are currently in climb mode or not.
   */
  public ClimbTeleop(Climber climber, XboxController controller, BooleanSupplier climb_mode) {
    // Assign member variables.
    climber_ = climber;
    controller_ = controller;
    climb_mode_ = climb_mode;

    // Set subsystem requirements.
    addRequirements(climber_);
  }

  @Override
  public void initialize() {
    // Undo brake.
    climber_.setBrake(false);
  }

  @Override
  public void execute() {
    // If we are not in climb mode, just set the pivots and exit.
    if (!climb_mode_.getAsBoolean()) {
      climber_.setLeftPercent(0);
      climber_.setRightPercent(0);
      climber_.setPivot(true, true);
      return;
    }

    // Use left trigger to move climber down and left bumper to move climber up.
    if (Math.abs(controller_.getLeftTriggerAxis()) > 0.1 || controller_.getLeftBumper()) {
      climber_.setLeftPercent(
          controller_.getLeftBumper() ? 0.4 : -controller_.getLeftTriggerAxis());

      // In manual control, so we don't have a position setpoint to hold the arm in place.
      if (left_arm_setpoint_set_)
        left_arm_setpoint_set_ = false;
    } else {
      // Hold the left arm position with closed loop control.
      if (!left_arm_setpoint_set_) {
        climber_.setLeftPosition(climber_.getLeftPosition());
        left_arm_setpoint_set_ = true;
      }
    }

    // Use right trigger to move climber down and right bumper to move climber up.
    if (Math.abs(controller_.getRightTriggerAxis()) > 0.1 || controller_.getRightBumper()) {
      climber_.setRightPercent(
          controller_.getRightBumper() ? 0.4 : -controller_.getRightTriggerAxis());

      // In manual control, so we don't have a position setpoint to hold the arm in place.
      if (right_arm_setpoint_set_)
        right_arm_setpoint_set_ = false;
    } else {
      // Hold the right arm position with closed loop control.
      if (!right_arm_setpoint_set_) {
        climber_.setRightPosition(climber_.getRightPosition());
        right_arm_setpoint_set_ = true;
      }
    }

    // Use Y and A to toggle left and right pivots respectively.
    if (controller_.getYButtonPressed())
      climber_.setPivot(!climber_.getLeftPivot(), climber_.getRightPivot());
    if (controller_.getAButtonPressed())
      climber_.setPivot(climber_.getLeftPivot(), !climber_.getRightPivot());
  }
}
