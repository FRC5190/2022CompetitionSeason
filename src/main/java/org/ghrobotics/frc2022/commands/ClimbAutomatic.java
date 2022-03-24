package org.ghrobotics.frc2022.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import java.util.function.BooleanSupplier;
import org.ghrobotics.frc2022.subsystems.Climber;

public class ClimbAutomatic extends SequentialCommandGroup {

  public boolean waiting_ = false;

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
        new InstantCommand(() -> climber.setPivot(true, true)),
        new InstantCommand(() -> climber.setPivot(true, false)),
        new ClimbToPosition(climber, Constants.kMaxHeight, Constants.kReadyForL2Height)
            .withInterrupt(() ->
                Math.abs(climber.getRightPosition() - Constants.kReadyForL2Height) <
                    Climber.Constants.kErrorTolerance),

        setWaiting(),
        new WaitUntilCommand(advance_button),
        setUnwaiting(),

        // Part 2: climb to mid rung: pull the right arm all the way down. When the left arm is
        // all the way up, un-pivot it.
        new ClimbToPosition(climber, Constants.kMaxHeight,
            Constants.kClimbHeight - Units.inchesToMeters(1)),
        new WaitCommand(0.75),
        new InstantCommand(() -> climber.setPivot(false, false)),
        setWaiting(),
        new WaitUntilCommand(advance_button),
        setUnwaiting(),

        // Part 3: climb to high rung: pull the left all arm the way down. When 8 inches from the
        // top, pivot right arm and extend to max height. Once the right arm reaches max height,
        // un-pivot it.
        new ClimbToPosition(climber, Constants.kSafeL3PivotHeight, Constants.kClimbHeight),
        new WaitCommand(3),
        new InstantCommand(() -> climber.setPivot(false, true)),
        new ClimbToPosition(climber, Constants.kClimbHeight, Constants.kClimbHeight),
        new ClimbToPosition(climber, Constants.kClimbHeight, Constants.kMaxHeight),
        setWaiting(),
        new WaitUntilCommand(advance_button),
        setUnwaiting(),
        new InstantCommand(() -> climber.setPivot(false, false)),
        setWaiting(),
        new WaitUntilCommand(advance_button),
        setUnwaiting(),

        // Part 4: climb to traversal rung: pull the right arm all the way down.
        new ClimbToPosition(climber, 0, Constants.kMaxHeight - Units.inchesToMeters(4)),
        new InstantCommand(() -> climber.setBrake(true)),
        new InstantCommand(() -> climber.setLeftPercent(0)),
        new InstantCommand(() -> climber.setRightPercent(0)),

        // Celebrate.
        new InstantCommand(climber::setOrchestra),
        new WaitCommand(10000)
    );

    // Set subsystem requirements.
    addRequirements(climber);
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
   * Returns the command to set the waiting flag to true.
   *
   * @return The command to set the waiting flag to true.
   */
  private Command setWaiting() {
    return new InstantCommand(() -> {
      waiting_ = true;
    });
  }

  /**
   * Returns the command to set the waiting flag to false.
   *
   * @return The command to set the waiting flag to false.
   */
  private Command setUnwaiting() {
    return new InstantCommand(() -> {
      waiting_ = false;
    });
  }

  public static class Constants {
    // Heights
    public static final double kMaxHeight = Climber.Constants.kMaxHeight;
    public static final double kClimbHeight = Units.inchesToMeters(0);
    public static final double kReadyForL2Height = Units.inchesToMeters(24);
    public static final double kSafeL3PivotHeight = kMaxHeight - Units.inchesToMeters(18);

    // Celebration
    public static final double kOrchestraWaitTime = 15;
  }
}
