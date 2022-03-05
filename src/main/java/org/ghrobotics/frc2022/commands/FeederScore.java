package org.ghrobotics.frc2022.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.BooleanSupplier;
import org.ghrobotics.frc2022.subsystems.Feeder;

public class FeederScore extends CommandBase {
  // Reference to subsystem and scoring supplier.
  private final Feeder feeder_;
  private final BooleanSupplier ready_to_fire_;

  /**
   * Runs the feeder to score cargo into the hub.
   *
   * @param feeder        Reference to feeder subsystem.
   * @param ready_to_fire Whether the scoring system is ready to accept a cargo.
   */
  public FeederScore(Feeder feeder, BooleanSupplier ready_to_fire) {
    // Assign member variables.
    feeder_ = feeder;
    ready_to_fire_ = ready_to_fire;

    // Add subsystem requirements.
    addRequirements(feeder);
  }

  @Override
  public void execute() {
    // Run the feeder while we are ready to fire.
    boolean ready = ready_to_fire_.getAsBoolean();
    feeder_.setWallPercent(ready ? 0.85 : 0);
    feeder_.setFloorPercent(ready ? 0.85 : 0);
  }

  @Override
  public void end(boolean interrupted) {
    // Set all percentages to 0.
    feeder_.setWallPercent(0);
    feeder_.setFloorPercent(0);
  }
}
