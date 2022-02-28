package org.ghrobotics.frc2022.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.ghrobotics.frc2022.subsystems.Climber;

public class ClimbTeleop extends CommandBase {
  // Reference to subsystem and controller.
  private final Climber climber_;
  private final XboxController controller_;

  // Store left arm setpoint for closed loop control.
  private boolean left_arm_setpoint_set_;

  /**
   * Controls the climber with manual inputs from the Xbox controller.
   *
   * @param climber    Reference to climber subsystem.
   * @param controller Reference to Xbox controller used to control the subsystem.
   */
  public ClimbTeleop(Climber climber, XboxController controller) {
    // Assign member variables.
    climber_ = climber;
    controller_ = controller;

    // Set subsystem requirements.
    addRequirements(climber_);
  }

  @Override
  public void execute() {
    // Use left stick y-axis to move the left climb arm up and down.
    if (Math.abs(controller_.getLeftY()) > 0.1) {
      climber_.setLeftPercent(-controller_.getLeftY());
      if (left_arm_setpoint_set_)
        left_arm_setpoint_set_ = false;
    } else {
      // Hold the left arm position with closed loop control.
      if (!left_arm_setpoint_set_) {
        climber_.setLeftPosition(climber_.getLeftPosition());
      }
    }

    // Use right stick y-axis to move the right climb arm up and down.
    if (Math.abs(controller_.getRightY()) > 0.1) {
      climber_.setBrake(false);
      climber_.setRightPercent(-controller_.getRightY());
    } else {
      // Hold the right arm position with brake.
      climber_.setBrake(true);
      climber_.setRightPercent(0);
    }

    // Use left and right bumpers to toggle pivot.
    if (controller_.getLeftBumperPressed())
      climber_.setPivot(!climber_.getLeftPivot(), climber_.getRightPivot());
    if (controller_.getRightBumperPressed())
      climber_.setPivot(climber_.getLeftPivot(), !climber_.getRightPivot());
  }
}
