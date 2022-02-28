package org.ghrobotics.frc2022.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.ghrobotics.frc2022.subsystems.Climber;

public class ClimbReset extends CommandBase {
  // Reference to subsystem.
  private final Climber climber_;

  /**
   * Resets the state of the climber, bringing both arms down until the limit.
   *
   * @param climber Reference to climber subsystem.
   */
  public ClimbReset(Climber climber) {
    // Assign member variables.
    climber_ = climber;

    // Set subsystem requirements.
    addRequirements(climber_);
  }

  @Override
  public void initialize() {
    // Disable soft limits.
    climber_.enableSoftLimits(false);

    // Undo brake.
    climber_.setBrake(false);

    // Pivot both arms out.
    climber_.setPivot(true, true);
  }

  @Override
  public void execute() {
    // Run each climber arm down until the limit switch is triggered.
    climber_.setLeftPercent(climber_.getLeftReverseLimitSwitchClosed() ? 0 : -0.1);
    climber_.setRightPercent(climber_.getRightReverseLimitSwitchClosed() ? 0 : -0.1);
  }

  @Override
  public void end(boolean interrupted) {
    // Re-enable soft limits and zero outputs.
    climber_.enableSoftLimits(true);
    climber_.setLeftPercent(0);
    climber_.setRightPercent(0);
    climber_.setBrake(true);
  }

  @Override
  public boolean isFinished() {
    // We are done when both limit switches are active.
    return climber_.getLeftReverseLimitSwitchClosed() && climber_.getRightReverseLimitSwitchClosed();
  }
}
