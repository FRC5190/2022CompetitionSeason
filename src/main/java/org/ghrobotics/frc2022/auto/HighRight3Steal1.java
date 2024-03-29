package org.ghrobotics.frc2022.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import java.util.List;
import org.ghrobotics.frc2022.RobotState;
import org.ghrobotics.frc2022.Superstructure;
import org.ghrobotics.frc2022.commands.DriveTrajectory;
import org.ghrobotics.frc2022.subsystems.Drivetrain;

/**
 * This auto mode runs the following routine:
 * - 1 cargo must be preloaded
 * - Score 1
 * - Pick up bottom friendly alliance cargo
 * - Pick up middle friendly alliance cargo
 * - Score 2
 * - Pick up middle enemy alliance cargo
 * - Eject 1
 */
public class HighRight3Steal1 extends SequentialCommandGroup {
  // Paths
  private final Trajectory path1;
  private final Trajectory path2;
  private final Trajectory path3;
  private final Trajectory path4;

  public HighRight3Steal1(RobotState robot_state, Drivetrain drivetrain,
                          Superstructure superstructure) {

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
        new Pose2d(4.595, 2.981, Rotation2d.fromDegrees(109)),
        AutoConfig.kForwardConfig);

    addCommands(
        // Reset odometry.
        new InstantCommand(() -> robot_state.resetPosition(path1.getInitialPose())),

        // Wait for climb arms to pivot.
        new WaitCommand(0.5),

        // Score.
        superstructure.scoreHighGoal().withTimeout(2.5),

        // Follow trajectories while intaking.
        new ParallelRaceGroup(
            new SequentialCommandGroup(
                new DriveTrajectory(drivetrain, robot_state, path1),
                new DriveTrajectory(drivetrain, robot_state, path2),
                new DriveTrajectory(drivetrain, robot_state, path3)
            ),
            superstructure.intake()
        ),

        // Score
        superstructure.scoreHighGoal().withTimeout(3),

        // Follow trajectory while intaking.
        new ParallelRaceGroup(
            new DriveTrajectory(drivetrain, robot_state, path4),
            superstructure.intake()
        ),

        // Eject.
        superstructure.eject().withTimeout(4)
    );
  }
}
