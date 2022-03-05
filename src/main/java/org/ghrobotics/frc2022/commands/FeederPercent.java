package org.ghrobotics.frc2022.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.ghrobotics.frc2022.subsystems.Feeder;

public class FeederPercent extends CommandBase {
  // Reference to subsystem and percent setpoints.
  private final Feeder feeder_;
  private final double wall_percent_;
  private final double floor_percent_;

  /**
   * Sets the feeder percent in [-1, 1].
   *
   * @param feeder        Reference to feeder subsystem.
   * @param wall_percent  The desired wall percent.
   * @param floor_percent The desired floor percent.
   */
  public FeederPercent(Feeder feeder, double wall_percent, double floor_percent) {
    // Assign member variables.
    feeder_ = feeder;
    wall_percent_ = wall_percent;
    floor_percent_ = floor_percent;

    // Add subsystem requirements.
    addRequirements(feeder_);
  }

  @Override
  public void initialize() {
    feeder_.setWallPercent(wall_percent_);
    feeder_.setFloorPercent(floor_percent_);
  }

  @Override
  public void end(boolean interrupted) {
    feeder_.setWallPercent(0);
    feeder_.setFloorPercent(0);
  }
}
