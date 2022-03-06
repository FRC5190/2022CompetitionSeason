package org.ghrobotics.frc2022.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import java.util.function.BooleanSupplier;
import org.ghrobotics.frc2022.subsystems.Climber;

public class ClimbAutomatic extends SequentialCommandGroup {
  /**
   * Automatically climbs to the traversal rung.
   *
   * @param climber        Reference to climber subsystem.
   * @param advance_button Supplier to advance state when necessary.
   */
  public ClimbAutomatic(Climber climber, BooleanSupplier advance_button) {
    // Create the automatic climb routine.
    addCommands(
        // Part 1: move the arms to be ready for mid rung climb.
        new InstantCommand(() -> climber.setPivot(true, false)),
        new ClimbToPosition(climber, Constants.kMaxHeight, Constants.kReadyForL2Height)
            .withInterrupt(() ->
                Math.abs(climber.getRightPosition() - Constants.kReadyForL2Height) <
                    Climber.Constants.kErrorTolerance),

        new WaitUntilCommand(advance_button),

        // Part 2: climb to mid rung: pull the right arm all the way down. When the left arm is
        // all the way up, un-pivot it.
        new ClimbToPosition(climber, Constants.kMaxHeight, Constants.kClimbHeight),
        new InstantCommand(() -> climber.setPivot(false, false)),
        new WaitUntilCommand(advance_button),

        // Part 3: climb to high rung: pull the left all arm the way down. When 8 inches from the
        // top, pivot right arm and extend to max height. Once the right arm reaches max height,
        // un-pivot it.
        new ClimbToPosition(climber, Constants.kSafeL3PivotHeight, Constants.kClimbHeight),
        new InstantCommand(() -> climber.setPivot(false, true)),
        new ClimbToPosition(climber, Constants.kClimbHeight, Constants.kMaxHeight),
        new InstantCommand(() -> climber.setPivot(false, false)),
        new WaitUntilCommand(advance_button),

        // Part 4: climb to traversal rung: pull the right arm all the way down.
        new ClimbToPosition(climber, 0, Constants.kClimbHeight),
        new InstantCommand(() -> climber.setBrake(true)),

        // Celebrate.
        new InstantCommand(climber::setOrchestra),
        new WaitCommand(Constants.kOrchestraWaitTime)
    );

    // Set subsystem requirements.
    addRequirements(climber);
  }

  public static class Constants {
    // Heights
    public static final double kMaxHeight = Climber.Constants.kMaxHeight;
    public static final double kClimbHeight = Units.inchesToMeters(0);
    public static final double kReadyForL2Height = Units.inchesToMeters(17);
    public static final double kSafeL3PivotHeight = kMaxHeight - Units.inchesToMeters(7);

    // Celebration
    public static final double kOrchestraWaitTime = 15;
  }
}
