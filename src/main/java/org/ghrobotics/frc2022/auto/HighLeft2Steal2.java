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
 * - Shoot preloaded cargo
 * - Pick up top enemy alliance cargo
 * - Eject 1 in hangar
 * - Pick up top friendly alliance cargo
 * - Score 1
 * - Pick up middle enemy alliance cargo
 * - Eject 1 in hangar
 */
public class HighLeft2Steal2 extends SequentialCommandGroup {
  // Paths
  private final Trajectory path1;
  private final Trajectory path2;
  private final Trajectory path3;

  public HighLeft2Steal2(RobotState robot_state, Drivetrain drivetrain,
                         Superstructure superstructure) {
    // Create trajectories.
    path1 = TrajectoryGenerator.generateTrajectory(
        new Pose2d(6.159, 5.209, Rotation2d.fromDegrees(136.5)), List.of(),
        new Pose2d(6.122, 6.937, Rotation2d.fromDegrees(116)),
        AutoConfig.kForwardConfig);

    path2 = TrajectoryGenerator.generateTrajectory(
        new Pose2d(6.122, 6.937, Rotation2d.fromDegrees(116)), List.of(),
        new Pose2d(5.108, 6.623, Rotation2d.fromDegrees(239)),
        AutoConfig.kForwardConfig);

    path3 = TrajectoryGenerator.generateTrajectory(
        new Pose2d(5.108, 6.623, Rotation2d.fromDegrees(239)), List.of(),
        new Pose2d(4.419, 3.594, Rotation2d.fromDegrees(280)),
        AutoConfig.kForwardConfig);

    addCommands(
        // Reset odometry.
        new InstantCommand(() -> robot_state.resetPosition(path1.getInitialPose())),

        // Wait for climb arms to pivot.
        new WaitCommand(0.5),

        // Score
        superstructure.scoreHighGoal().withTimeout(2),

        // Follow trajectory while intaking.
        new ParallelRaceGroup(
            new DriveTrajectory(drivetrain, robot_state, path1),
            superstructure.intake()
        ),

        // Eject
        superstructure.eject().withTimeout(2),

        // Follow trajectory while intaking.
        new ParallelRaceGroup(
            new DriveTrajectory(drivetrain, robot_state, path2),
            superstructure.intake()
        ),

        // Score
        superstructure.scoreHighGoal().withTimeout(2),

        // Follow trajectory while intaking.
        new ParallelRaceGroup(
            new DriveTrajectory(drivetrain, robot_state, path3),
            superstructure.intake()
        ),

        // Eject
        superstructure.eject().withTimeout(2)
    );
  }
}
