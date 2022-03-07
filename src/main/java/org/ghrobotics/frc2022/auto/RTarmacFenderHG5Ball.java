package org.ghrobotics.frc2022.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.ghrobotics.frc2022.RobotState;
import org.ghrobotics.frc2022.Superstructure;
import org.ghrobotics.frc2022.commands.DriveTrajectory;
import org.ghrobotics.frc2022.subsystems.Drivetrain;

public class RTarmacFenderHG5Ball extends SequentialCommandGroup {
  // Trajectories
  private Trajectory fender_to_ball_2_;
  private Trajectory ball_2_to_intermediate_1_;
  private Trajectory intermediate_1_to_ball_3_;
  private Trajectory ball_3_to_hp_;
  private Trajectory hp_to_intermediate_2_;

  // Start Pose
  private Pose2d start_pose_;

  public RTarmacFenderHG5Ball(RobotState robot_state, Drivetrain drivetrain,
                              Superstructure superstructure) {
    // Create routine.
    addCommands(
        // Reset odometry.
        new InstantCommand(() -> robot_state.resetPosition(start_pose_)),

        // Go to ball 2 while shooting ball 1. Shooting ball 1 ends when the trajectory to ball 2
        // is done with traversal.
        new ParallelRaceGroup(
            new DriveTrajectory(drivetrain, robot_state, fender_to_ball_2_),
            superstructure.scoreHighGoal()),

        // Go to ball 3 while continuously running the intake. This should pick up ball 2 and 3.
        new ParallelRaceGroup(
            new SequentialCommandGroup(
                new DriveTrajectory(drivetrain, robot_state, ball_2_to_intermediate_1_),
                new DriveTrajectory(drivetrain, robot_state, intermediate_1_to_ball_3_)),
            superstructure.intake()),

        // Go to ball 4 and 5 while scoring balls 2 and 3.
        new ParallelRaceGroup(
            new DriveTrajectory(drivetrain, robot_state, ball_3_to_hp_),
            superstructure.scoreHighGoal()),

        // Intake (for an extended period of time).
        superstructure.intake().withTimeout(1.5),

        // Come back and score.
        new ParallelCommandGroup(
            new DriveTrajectory(drivetrain, robot_state, hp_to_intermediate_2_),
            superstructure.scoreHighGoal())
    );
  }
}
