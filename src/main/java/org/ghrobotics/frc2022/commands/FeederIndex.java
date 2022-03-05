package org.ghrobotics.frc2022.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.ghrobotics.frc2022.subsystems.Feeder;

public class FeederIndex extends CommandBase {
  // Reference to subsystem.
  private final Feeder feeder_;

  /**
   * Indexes cargo in the feeder and maintains appropriate spacing for feeding into the turret
   * shooter. This uses the photoelectric sensors on the feeder to space cargo.
   *
   * @param feeder Reference to feeder subsystem.
   */
  public FeederIndex(Feeder feeder) {
    // Assign member variables.
    feeder_ = feeder;

    // Add subsystem requirements.
    addRequirements(feeder_);
  }

  @Override
  public void execute() {
    // Check if the top sensor is triggered. If so, there is at least one cargo in the feeder
    // already => don't run the wall.
    if (feeder_.getExitSensor()) {
      feeder_.setWallPercent(0);
      // Check if the bottom sensor is triggered. If it is, then the feeder is full, and we don't
      // run anything.
      if (feeder_.getIntakeSensor()) {
        feeder_.setFloorPercent(0);
      } else {
        // Only one ball is in the feeder, so we can run the floor.
        feeder_.setFloorPercent(0.75);
      }
    } else {
      // There is no cargo in the feeder / a cargo is stuck somewhere in the middle such that it
      // is not tripping the exit sensor. So run the wall and floor.
      feeder_.setWallPercent(0.75);
      feeder_.setFloorPercent(0.75);
    }
  }

  @Override
  public void end(boolean interrupted) {
    // Set both floor and wall percentages to 0.
    feeder_.setWallPercent(0);
    feeder_.setFloorPercent(0);
  }
}
