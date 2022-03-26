package org.ghrobotics.frc2022.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.BooleanSupplier;
import org.ghrobotics.frc2022.subsystems.CargoTracker;
import org.ghrobotics.frc2022.subsystems.Intake;

public class IntakeAutomatic extends CommandBase {
  // References to subsystem, and when to intake/score.
  private final Intake intake_;
  private final CargoTracker cargo_tracker_;
  private final BooleanSupplier wants_intake_;
  private final BooleanSupplier wants_score_;

  /**
   * Runs the intake automatically based on sensor input.
   *
   * @param intake       Reference to intake subsystem.
   * @param wants_intake Whether we want to intake currently.
   * @param wants_score  Whether we want to score currently.
   */
  public IntakeAutomatic(Intake intake, CargoTracker cargo_tracker,
                         BooleanSupplier wants_intake,
                         BooleanSupplier wants_score) {
    // Assign member variables.
    intake_ = intake;
    cargo_tracker_ = cargo_tracker;
    wants_intake_ = wants_intake;
    wants_score_ = wants_score;

    // Add subsystem requirements.
    addRequirements(intake_);
  }

  @Override
  public void execute() {
    // Check if we want to intake.
    if (wants_intake_.getAsBoolean()) {
      // Extend intake out.
      intake_.setPivot(true);

      // Run intake at desired speed.
      intake_.setIntakePercent(Constants.kCollectionIntakeSpeed);

      // Run indexing algorithm.
      if (!cargo_tracker_.getBallAtUpper()) {
        intake_.setFloorPercent(Constants.kIndexFeederSpeed);
        intake_.setWallPercent(Constants.kIndexFeederSpeed);
      } else {
        intake_.setWallPercent(0);
        intake_.setFloorPercent(cargo_tracker_.getBallAtLower() ? 0 : Constants.kIndexFeederSpeed);
      }
    } else {
      // If we don't need the intake, pivot it back.
      intake_.setPivot(false);
      intake_.setIntakePercent(0);
      intake_.setFloorPercent(0);
      intake_.setWallPercent(0);
    }

    // Check if we want to score.
    if (wants_score_.getAsBoolean()) {
      // Run feeder motors at desired speeds.
      intake_.setFloorPercent(Constants.kScoreFeederSpeed);
      intake_.setWallPercent(Constants.kScoreFeederSpeed);

      // Run intake motor at a slower speed to help with feeder.
      intake_.setIntakePercent(Constants.kScoreIntakeSpeed);
    }
  }

  @Override
  public void end(boolean interrupted) {
    // Set all outputs to 0.
    intake_.setIntakePercent(0);
    intake_.setFloorPercent(0);
    intake_.setWallPercent(0);
    intake_.setPivot(false);
  }

  public static class Constants {
    public static final double kCollectionIntakeSpeed = 1.0;
    public static final double kScoreIntakeSpeed = 0.5;
    public static final double kScoreFeederSpeed = 0.85;
    public static final double kIndexFeederSpeed = 0.55;
  }
}
