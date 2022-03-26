package org.ghrobotics.frc2022.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
 * - Pick up top friendly alliance cargo
 * - Score 2
 * - Pick up 2 cargo from human player station
 * - Come back to scoring area
 * - Score 2
 */
public class HighLeft4 extends SequentialCommandGroup {
  // Paths
  private final Trajectory path1;
  private final Trajectory path2;
  private final Trajectory path3;

  public HighLeft4(RobotState robot_state, Drivetrain drivetrain, Superstructure superstructure) {
    // Create trajectories.
    path1 = TrajectoryGenerator.generateTrajectory(
        new Pose2d(6.159, 5.209, Rotation2d.fromDegrees(136.5)), List.of(),
        new Pose2d(5.258, 5.985, Rotation2d.fromDegrees(136.5)),
        AutoConfig.kForwardConfig);

    path2 = TrajectoryGenerator.generateTrajectory(
        new Pose2d(5.258, 5.985, Rotation2d.fromDegrees(136.5)),
        List.of(new Translation2d(4.207, 5.159)),
        new Pose2d(1.453, 1.483, Rotation2d.fromDegrees(225)),
        AutoConfig.kForwardConfig);

    path3 = TrajectoryGenerator.generateTrajectory(
        new Pose2d(1.453, 1.483, Rotation2d.fromDegrees(225)), List.of(),
        new Pose2d(4.419, 4.521, Rotation2d.fromDegrees(180)),
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
        superstructure.scoreHighGoal().withTimeout(3),

        // Follow trajectory while intaking.
        new ParallelRaceGroup(
            new DriveTrajectory(drivetrain, robot_state, path2),
            superstructure.intake()
        ),

        // Go to scoring area.
        new DriveTrajectory(drivetrain, robot_state, path3),

        // Score
        superstructure.scoreHighGoal().withTimeout(3)
    );
  }
}
