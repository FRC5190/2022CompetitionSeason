package org.ghrobotics.frc2022.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.ghrobotics.frc2022.subsystems.Climber;

public class ClimbToPosition extends CommandBase {
  // Reference to subsystem and setpoints.
  private final Climber climber_;
  private final double l_setpoint_;
  private final double r_setpoint_;

  /**
   * Moves the climber arms to a specified position.
   *
   * @param climber    Reference to climber subsystem.
   * @param l_setpoint The left setpoint in meters.
   * @param r_setpoint The right setpoint in meters.
   */
  public ClimbToPosition(Climber climber, double l_setpoint, double r_setpoint) {
    // Assign member variables.
    climber_ = climber;
    l_setpoint_ = l_setpoint;
    r_setpoint_ = r_setpoint;

    // Set subsystem requirements.
    addRequirements(climber_);
  }

  @Override
  public void initialize() {
    // Undo brake.
    climber_.setBrake(false);

    // Set position setpoints on climber arms.
    climber_.setLeftPosition(l_setpoint_);
    climber_.setRightPosition(r_setpoint_);
  }

  @Override
  public boolean isFinished() {
    // End the command when both arms are within tolerance of the given setpoint.
    return Math.abs(climber_.getLeftPosition() - l_setpoint_) < Climber.Constants.kErrorTolerance &&
        Math.abs(climber_.getRightPosition() - r_setpoint_) < Climber.Constants.kErrorTolerance;
  }
}
