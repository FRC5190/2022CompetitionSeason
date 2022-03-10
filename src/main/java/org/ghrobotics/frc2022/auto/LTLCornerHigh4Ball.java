package org.ghrobotics.frc2022.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.ghrobotics.frc2022.RobotState;
import org.ghrobotics.frc2022.Superstructure;
import org.ghrobotics.frc2022.commands.DriveTrajectory;
import org.ghrobotics.frc2022.subsystems.Drivetrain;

public class LTLCornerHigh4Ball extends SequentialCommandGroup {
  // Start at middle left corner of left tarmac; score 4 balls.
  public LTLCornerHigh4Ball(RobotState robot_state, Drivetrain drivetrain,
                            Superstructure superstructure) {
    // Create routine.
    addCommands(
        // Reset odometry.
        new InstantCommand(() -> robot_state.resetPosition(
            AutoPlanner.kLTarmacMLCornerToTopCargo.getInitialPose())),

        // Follow all trajectories while scoring.
        new ParallelRaceGroup(
            new SequentialCommandGroup(
                new DriveTrajectory(drivetrain, robot_state,
                    AutoPlanner.kLTarmacMLCornerToTopCargo),
                new DriveTrajectory(drivetrain, robot_state,
                    AutoPlanner.kTopCargoToHPCargo),
                new DriveTrajectory(drivetrain, robot_state,
                    AutoPlanner.kHPCargoToLeftScoringLocation)
            ),
            superstructure.scoreHighGoal(() -> true, () ->
                AutoPlanner.kTopCargoRegion.isPoseInRegion(robot_state.getRobotPose()))
        ),

        // Score final balls.
        superstructure.scoreHighGoal().withTimeout(2.5)
    );
  }
}
