package org.ghrobotics.frc2022.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.ghrobotics.frc2022.RobotState;
import org.ghrobotics.frc2022.Superstructure;
import org.ghrobotics.frc2022.commands.DriveTrajectory;
import org.ghrobotics.frc2022.subsystems.Drivetrain;

public class LTLCorner2Score2Eject extends SequentialCommandGroup {

  public LTLCorner2Score2Eject(RobotState robot_state, Drivetrain drivetrain,
                               Superstructure superstructure) {
    // Create routine.
    addCommands(
        // Reset odometry.
        new InstantCommand(() -> robot_state.resetPosition(
            AutoPlanner.kLTarmacMLCornerToTopStealCargo.getInitialPose())),

        // Score first ball.
        superstructure.scoreHighGoal().withTimeout(2),

        // Follow trajectories while intaking. Wait 2 seconds between each trajectory to score.
        new ParallelRaceGroup(
            new DriveTrajectory(drivetrain, robot_state,
                AutoPlanner.kLTarmacMLCornerToTopStealCargo),
            superstructure.intake()
        ),
        superstructure.eject().withTimeout(2),
        new ParallelRaceGroup(
            new DriveTrajectory(drivetrain, robot_state,
                AutoPlanner.kTopStealCargoToTopCargo),
            superstructure.intake()
        ),
        superstructure.scoreHighGoal().withTimeout(2),
        new ParallelRaceGroup(
            new DriveTrajectory(drivetrain, robot_state,
                AutoPlanner.kTopCargoToMiddleStealCargo),
            superstructure.intake()
        ),
        superstructure.eject().withTimeout(2)
    );
  }
}
