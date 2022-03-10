package org.ghrobotics.frc2022.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.ghrobotics.frc2022.RobotState;
import org.ghrobotics.frc2022.Superstructure;
import org.ghrobotics.frc2022.commands.DriveTrajectory;
import org.ghrobotics.frc2022.subsystems.Drivetrain;

public class LTLCornerHigh2Ball extends SequentialCommandGroup {
  // Start at middle left corner of left tarmac; score 2 balls.
  public LTLCornerHigh2Ball(RobotState robot_state, Drivetrain drivetrain,
                            Superstructure superstructure) {
    // Create routine.
    addCommands(
        // Reset odometry.
        new InstantCommand(() -> robot_state.resetPosition(
            AutoPlanner.kLTarmacMLCornerToTopCargo.getInitialPose())),

        // Pick up 2nd ball.
        new ParallelRaceGroup(
            new DriveTrajectory(drivetrain, robot_state, AutoPlanner.kLTarmacMLCornerToTopCargo),
            superstructure.intake()
        ),

        // Score
        superstructure.scoreHighGoal().withTimeout(3)
    );
  }
}
