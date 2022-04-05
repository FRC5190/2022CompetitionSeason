package org.ghrobotics.frc2022.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import java.util.function.BooleanSupplier;
import org.ghrobotics.frc2022.subsystems.Climber;

public class ClimbAutomatic extends SequentialCommandGroup {
  // Reference to subsystem and advance button.
  private final Climber climber_;
  private final BooleanSupplier advance_step_;

  // Keeps track of whether we are waiting for the driver.
  private boolean waiting_ = false;

  public ClimbAutomatic(Climber climber, BooleanSupplier advance_step) {
    // Assign member variables.
    climber_ = climber;
    advance_step_ = advance_step;

    // Create the routine.
    addCommands(
        // Reset the waiting flag to false (in case we are running this command multiple times).
        new InstantCommand(() -> waiting_ = false),

        /* PART 1: PREPARE TO CLIMB L2 */
        piv(true, true),
        pos(25.50, 24.00),
        piv(false, false),
        waitForAdvance(),

        /* PART 2: CLIMB L2 AND PREPARE TO CLIMB L3 */
        piv(false, true),
        pos(25.50, -1.50),
        piv(true, true),
        waitForAdvance(),

        /* PART 3: CLIMB L3 AND PREPARE TO CLIMB L4 */
        new ParallelCommandGroup(
            pos(-1.00, 25.50),
            new SequentialCommandGroup(
                new WaitCommand(1.5),
                piv(true, false)
            )
        ),
        waitForAdvance(),

        /* PART 4: CLIMB L4 */
        piv(true, true),
        pos(00.00, 22.00),
        piv(true, false),

        /* PART 5: CLEAN UP */
        new InstantCommand(() -> climber_.setBrake(true)),
        new InstantCommand(() -> {
          climber_.setLeftPercent(0);
          climber_.setRightPercent(0);
        }),
        new InstantCommand(climber::setOrchestra),
        new WaitCommand(10000)
    );
  }

  /**
   * Returns whether the command is waiting for driver input.
   *
   * @return Whether the command is waiting for driver input.
   */
  public boolean isWaiting() {
    return waiting_;
  }

  /**
   * Returns the command to take the climber arms to the provided positions.
   *
   * @param l The left arm setpoint in inches.
   * @param r The right arm setpoint in inches.
   * @return The command to take the climber arms to the provided positions.
   */
  private Command pos(double l, double r) {
    return new ClimbToPosition(climber_, Units.inchesToMeters(l), Units.inchesToMeters(r));
  }

  /**
   * Returns the command to pivot the climber arms.
   *
   * @param l The pivot value for the left arm.
   * @param r The pivot value for the right arm.
   * @return The command to pivot the climber arms.
   */
  private Command piv(boolean l, boolean r) {
    return new InstantCommand(() -> climber_.setPivot(l, r));
  }

  /**
   * Returns the command to wait for the driver to advance step.
   *
   * @return The command to wait for the driver to advance step.
   */
  private Command waitForAdvance() {
    return new SequentialCommandGroup(
        new InstantCommand(() -> waiting_ = true),
        new WaitUntilCommand(advance_step_),
        new InstantCommand(() -> waiting_ = false)
    );
  }
}
