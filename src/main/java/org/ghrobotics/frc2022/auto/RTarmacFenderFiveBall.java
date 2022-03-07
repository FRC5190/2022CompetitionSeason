package org.ghrobotics.frc2022.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.ghrobotics.frc2022.RobotState;
import org.ghrobotics.frc2022.Superstructure;
import org.ghrobotics.frc2022.commands.DriveTrajectory;
import org.ghrobotics.frc2022.subsystems.Drivetrain;

public class RTarmacFenderFiveBall extends SequentialCommandGroup {
  // Trajectories
  private Trajectory fender_to_ball_2_;
  private Trajectory ball_2_to_intermediate_1_;
  private Trajectory intermediate_1_to_ball_3_;
  private Trajectory ball_3_to_hp_;
  private Trajectory hp_to_intermediate_2_;

  // Start Pose
  private Pose2d start_pose_;

  public RTarmacFenderFiveBall(RobotState robot_state, Drivetrain drivetrain,
                               Superstructure superstructure) {
    // Create routine.
    addCommands(
        // Reset odometry.
        new InstantCommand(() -> robot_state.resetPosition(start_pose_)),

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
                    AutoPlanner.kHPCargoToMiddleCargo)
            ),
            superstructure.scoreHighGoal()
        )
    );
  }
}
