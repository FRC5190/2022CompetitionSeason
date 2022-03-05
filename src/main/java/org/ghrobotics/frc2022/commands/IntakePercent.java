package org.ghrobotics.frc2022.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.ghrobotics.frc2022.subsystems.Intake;

public class IntakePercent extends CommandBase {
  // Reference to subsystem, percent setpoint, and intake keep-down flag.
  private final Intake intake_;
  private final double percent_;
  private final boolean keep_intake_down_;

  /**
   * Sets the intake and bridge motors to the given percent in [-1, 1].
   *
   * @param intake           Reference to intake subsystem.
   * @param percent          The percent setpoint for the motors.
   * @param keep_intake_down Whether to keep the intake down after the command ends.
   */
  public IntakePercent(Intake intake, double percent, boolean keep_intake_down) {
    // Assign member variables.
    intake_ = intake;
    percent_ = percent;
    keep_intake_down_ = keep_intake_down;

    // Add subsystem requirements.
    addRequirements(intake_);
  }

  /**
   * Sets the intake and bridge motors to the given percent in [-1, 1] and pivots the intake back
   * up when done.
   *
   * @param intake  Reference to intake subsystem.
   * @param percent The percent setpoint for the motors.
   */
  public IntakePercent(Intake intake, double percent) {
    this(intake, percent, false);
  }

  @Override
  public void initialize() {
    // Make sure the intake is pivoted down.
    intake_.setPivot(true);

    // Set the percent setpoint.
    intake_.setPercent(percent_);
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the motor and pivot the intake back up if we want to.
    intake_.setPercent(0);
    intake_.setPivot(keep_intake_down_);
  }
}
