package org.ghrobotics.frc2022.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.BooleanSupplier;
import org.ghrobotics.frc2022.subsystems.Feeder;

public class FeederIndex extends CommandBase {
  // Reference to subsystem and whether we should be scoring.
  private final Feeder feeder_;
  private final BooleanSupplier scoring_;

  /**
   * Handles all feeder motion while intaking and/or scoring. While scoring is not required, the
   * feeder will index the balls with appropriate spacing. When we want to score, the balls will
   * be fed into the turret shooter.
   *
   * @param feeder  Reference to feeder subsystem.
   * @param scoring Whether we should be scoring.
   */
  public FeederIndex(Feeder feeder, BooleanSupplier scoring) {
    // Assign member variables.
    feeder_ = feeder;
    scoring_ = scoring;

    // Add subsystem requirements.
    addRequirements(feeder_);
  }

  /**
   * Handles all feeder motion while intaking. This assumes scoring is not needed, meaning that
   * the feeder will only index the balls to maintain appropriate spacing.
   *
   * @param feeder Reference to feeder subsystem.
   */
  public FeederIndex(Feeder feeder) {
    this(feeder, () -> false);
  }

  @Override
  public void execute() {
    // Check if we should be scoring at this time.
    boolean scoring = scoring_.getAsBoolean();

    // If we should be scoring, run the feeder at the scoring percentage.
    if (scoring) {
      feeder_.setWallPercent(Constants.kScoringSpeed);
      feeder_.setFloorPercent(Constants.kScoringSpeed);
    } else {
      // We should be indexing the cargo and maintaining appropriate spacing.
      // If the upper exit sensor is not tripped, continue to run the feeder.
      if (!feeder_.getExitSensor()) {
        feeder_.setWallPercent(Constants.kIndexingSpeed);
        feeder_.setFloorPercent(Constants.kIndexingSpeed);
      } else {
        // The upper sensor is tripped, so don't run the wall anymore.
        feeder_.setWallPercent(0);

        // If the lower sensor is also tripped, we are full; don't run anything. If not, run the
        // floor motors.
        if (feeder_.getIntakeSensor())
          feeder_.setFloorPercent(0);
        else
          feeder_.setFloorPercent(Constants.kIndexingSpeed);
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    // Set all percentages to 0.
    feeder_.setWallPercent(0);
    feeder_.setFloorPercent(0);
  }

  public static class Constants {
    public static final double kScoringSpeed = 0.85;
    public static final double kIndexingSpeed = 0.75;
  }
}
