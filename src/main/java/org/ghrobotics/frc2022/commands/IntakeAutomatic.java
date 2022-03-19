package org.ghrobotics.frc2022.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.BooleanSupplier;
import org.ghrobotics.frc2022.subsystems.Intake;

public class IntakeAutomatic extends CommandBase {
  // References to subsystem, when to intake/score, and whether we want to stow intake.
  private final Intake intake_;
  private final BooleanSupplier wants_intake_;
  private final BooleanSupplier wants_score_;
  private final BooleanSupplier wants_stow_;

  /**
   * Runs the intake automatically based on sensor input.
   *
   * @param intake       Reference to intake subsystem.
   * @param wants_intake Whether we want to intake currently.
   * @param wants_score  Whether we want to score currently.
   * @param wants_stow   Whether we want to stow the intake when not being used.
   */
  public IntakeAutomatic(Intake intake, BooleanSupplier wants_intake, BooleanSupplier wants_score,
                         BooleanSupplier wants_stow) {
    // Assign member variables.
    intake_ = intake;
    wants_intake_ = wants_intake;
    wants_score_ = wants_score;
    wants_stow_ = wants_stow;

    // Add subsystem requirements.
    addRequirements(intake_);
  }

  @Override
  public void execute() {
    // Check whether we want to intake.
    if (wants_intake_.getAsBoolean()) {
      // Extend intake out.
      intake_.setPivot(true);

      // Run intake at desired speed.
      intake_.setIntakePercent(Constants.kCollectionIntakeSpeed);

      // Run indexing algorithm: if no sensors are tripped, run all feeder elements.
      if (!intake_.getExitSensor()) {
        intake_.setFloorPercent(Constants.kIndexFeederSpeed);
        intake_.setWallPercent(Constants.kIndexFeederSpeed);
      } else {
        // The upper sensor is tripped, so don't run the wall anymore.
        intake_.setWallPercent(0);

        // If the lower sensor is also tripped, we are full; don't run anything.
        intake_.setFloorPercent(intake_.getIntakeSensor() ? 0 : Constants.kIndexFeederSpeed);
      }
    } else {
      // If we aren't intaking, and want to stow the intake, do that.
      intake_.setPivot(!wants_stow_.getAsBoolean());
      intake_.setIntakePercent(0);
      intake_.setFloorPercent(0);
      intake_.setWallPercent(0);
    }

    // Check whether we want to score.
    if (wants_score_.getAsBoolean()) {
      // Run feeder wall and floor at feed rate.
      intake_.setFloorPercent(Constants.kScoreFeederSpeed);
      intake_.setWallPercent(Constants.kScoreFeederSpeed);

      // Run intake and intake speed for scoring.
      intake_.setPivot(false);
      intake_.setIntakePercent(Constants.kScoreIntakeSpeed);
    }
  }

  @Override
  public void end(boolean interrupted) {
    // Set all outputs to 0.
    intake_.setIntakePercent(0);
    intake_.setFloorPercent(0);
    intake_.setWallPercent(0);
    intake_.setPivot(!wants_stow_.getAsBoolean());
  }

  public static class Constants {
    public static final double kCollectionIntakeSpeed = 1.0;
    public static final double kScoreIntakeSpeed = 0.5;
    public static final double kScoreFeederSpeed = 0.55;
    public static final double kIndexFeederSpeed = 0.85;
  }
}
