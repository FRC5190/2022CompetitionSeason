package org.ghrobotics.frc2022.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.ghrobotics.frc2022.RobotState;
import org.ghrobotics.frc2022.Superstructure;
import org.ghrobotics.frc2022.commands.DriveTrajectory;
import org.ghrobotics.frc2022.subsystems.Drivetrain;

public class RTFender5Score extends SequentialCommandGroup {
  // Start at right tarmac up against fender; score 5 balls.
  public RTFender5Score(RobotState robot_state, Drivetrain drivetrain,
                        Superstructure superstructure) {
    // Create routine.
    addCommands(
        new WaitCommand(1.0),

        // Reset odometry.
        new InstantCommand(() -> robot_state.resetPosition(
            AutoPlanner.kRTarmacFenderWallToBottomCargo.getInitialPose())),

        // Follow pre-calculated trajectories while intaking and scoring.
        new ParallelCommandGroup(
            new SequentialCommandGroup(
                new DriveTrajectory(drivetrain, robot_state,
                    AutoPlanner.kRTarmacFenderWallToBottomCargo),
                new DriveTrajectory(drivetrain, robot_state,
                    AutoPlanner.kBottomCargoToIntermediateA),
                new DriveTrajectory(drivetrain, robot_state,
                    AutoPlanner.kIntermediateAToMiddleCargoToHPCargo),
                new DriveTrajectory(drivetrain, robot_state,
                    AutoPlanner.kHPCargoToRightScoringLocation)
            ),
            superstructure.scoreHighGoal(() -> true, () ->
                AutoPlanner.kBottomCargoRegion.isPoseInRegion(robot_state.getRobotPose()) ||
                    AutoPlanner.kMiddleCargoRegion.isPoseInRegion(robot_state.getRobotPose()))
        ),

        // Finish scoring.
        superstructure.scoreHighGoal().withTimeout(2.5)
    );
  }
}
