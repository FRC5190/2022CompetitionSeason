package org.ghrobotics.frc2022.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
        new InstantCommand(() -> climber.setPivot(false, false)),
        new ClimbToPosition(climber, 0, Constants.kReadyForL2Height),
        new WaitUntilCommand(advance_button),

        // Part 2: climb to mid rung: pull the right arm all the way down. When 2 inches from
        // bottom, pivot right arm and extend left arm to max height. Once the left arm reaches
        // max height, pivot it.
        new ClimbToPosition(climber, 0, Constants.kSafeL2PivotHeight),
        new InstantCommand(() -> climber.setPivot(false, true)),
        new ClimbToPosition(climber, Constants.kMaxHeight, Constants.kClimbHeight),
        new InstantCommand(() -> climber.setPivot(true, true)),
        new WaitUntilCommand(advance_button),

        // Part 3: climb to high rung: pull the left all arm the way down. When 4 inches from the
        // top, unpivot right arm and extend to max height. Once the right arm reaches max height,
        // pivot it.
        new ClimbToPosition(climber, Constants.kSafeL3PivotHeight, 0),
        new InstantCommand(() -> climber.setPivot(true, false)),
        new ClimbToPosition(climber, Constants.kClimbHeight, Constants.kMaxHeight),
        new InstantCommand(() -> climber.setPivot(true, true)),
        new WaitUntilCommand(advance_button),

        // Part 4: climb to traversal rung: pull the right arm all the way down, then unpivot it.
        new ClimbToPosition(climber, 0, Constants.kClimbHeight),
        new InstantCommand(() -> climber.setPivot(true, false)),

        // Celebrate.
        new InstantCommand(climber::setOrchestra)
    );

    // Set subsystem requirements.
    addRequirements(climber);
  }

  public static class Constants {
    // Heights
    public static final double kMaxHeight = Climber.Constants.kMaxHeight;
    public static final double kClimbHeight = Units.inchesToMeters(0.0);
    public static final double kReadyForL2Height = Units.inchesToMeters(7.0);
    public static final double kSafeL2PivotHeight = Units.inchesToMeters(2.0);
    public static final double kSafeL3PivotHeight = kMaxHeight - Units.inchesToMeters(4.0);
  }
}
