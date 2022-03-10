package org.ghrobotics.frc2022.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.ghrobotics.frc2022.RobotState;
import org.ghrobotics.frc2022.Superstructure;
import org.ghrobotics.frc2022.subsystems.Drivetrain;

public class RTFenderLow1Ball extends SequentialCommandGroup {
  // Start at right tarmac up against fender; score 1 ball in low goal and exit.
  public RTFenderLow1Ball(RobotState robot_state, Drivetrain drivetrain,
                          Superstructure superstructure) {
    // Create routine.
    addCommands(
        // Reset odometry.
        new InstantCommand(() -> robot_state.resetPosition(
            AutoPlanner.kRTarmacFenderWallToBottomCargo.getInitialPose()
        )),
        // Score ball.
        superstructure.scoreLowGoalFender().withTimeout(4.0)
    );
  }
}
