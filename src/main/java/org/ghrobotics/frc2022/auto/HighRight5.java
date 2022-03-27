package org.ghrobotics.frc2022.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import java.util.List;
import org.ghrobotics.frc2022.RobotState;
import org.ghrobotics.frc2022.Superstructure;
import org.ghrobotics.frc2022.commands.DriveTrajectory;
import org.ghrobotics.frc2022.subsystems.Drivetrain;

/**
 * This auto mode runs the following routine:
 * - 1 cargo must be preloaded
 * - Pick up bottom friendly alliance cargo
 * - Score 2
 * - Pick up middle friendly alliance cargo
 * - Score 1
 * - Pick up 2 HP cargo
 * - Score 2
 */
public class HighRight5 extends SequentialCommandGroup {
  // Paths
  private final Trajectory path1;
  private final Trajectory path2;
  private final Trajectory path3;
  private final Trajectory path4;
  private final Trajectory path5;

  public HighRight5(RobotState robot_state, Drivetrain drivetrain, Superstructure superstructure) {
    // Create trajectories.
    path1 = TrajectoryGenerator.generateTrajectory(
        new Pose2d(7.624, 1.880, Rotation2d.fromDegrees(271.5)), List.of(),
        new Pose2d(7.602, 1.030, Rotation2d.fromDegrees(270)),
        AutoConfig.kForwardConfig);

    path2 = TrajectoryGenerator.generateTrajectory(
        new Pose2d(7.602, 1.030, Rotation2d.fromDegrees(270)), List.of(),
        new Pose2d(8.298, 1.256, Rotation2d.fromDegrees(182)),
        AutoConfig.kReverseConfig);

    path3 = TrajectoryGenerator.generateTrajectory(
        new Pose2d(8.298, 1.256, Rotation2d.fromDegrees(182)), List.of(),
        new Pose2d(5.196, 1.992, Rotation2d.fromDegrees(132)),
        AutoConfig.kForwardConfig);

    path4 = TrajectoryGenerator.generateTrajectory(
        new Pose2d(5.196, 1.992, Rotation2d.fromDegrees(132)), List.of(),
        new Pose2d(1.453, 1.483, Rotation2d.fromDegrees(225)),
        AutoConfig.kForwardConfig);

    path5 = TrajectoryGenerator.generateTrajectory(
        new Pose2d(1.453, 1.483, Rotation2d.fromDegrees(225)), List.of(),
        new Pose2d(5.056, 1.910, Rotation2d.fromDegrees(190)),
        AutoConfig.kReverseConfig);

    addCommands(
        // Reset odometry.
        new InstantCommand(() -> robot_state.resetPosition(path1.getInitialPose())),

        // Follow trajectory while intaking.
        new ParallelRaceGroup(
            new DriveTrajectory(drivetrain, robot_state, path1),
            superstructure.intake()
        ),

        // Score
        superstructure.scoreHighGoal().withTimeout(3.5),

        // Follow trajectory while intaking.
        new ParallelRaceGroup(
            new SequentialCommandGroup(
                new DriveTrajectory(drivetrain, robot_state, path2),
                new DriveTrajectory(drivetrain, robot_state, path3)
            ),
            superstructure.trackGoalWithTurret(),
            superstructure.intake()
        ),

        // Score
        superstructure.scoreHighGoal().withTimeout(3),

        // Follow trajectory while intaking.
        new ParallelRaceGroup(
            new SequentialCommandGroup(
                new DriveTrajectory(drivetrain, robot_state, path4),
                new DriveTrajectory(drivetrain, robot_state, path5)
            ),
            superstructure.intake()
        ),

        // Score
        superstructure.scoreHighGoal().withTimeout(3)
    );
  }
}
