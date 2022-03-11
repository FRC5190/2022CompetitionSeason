package org.ghrobotics.frc2022.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import org.ghrobotics.frc2022.RobotState;
import org.ghrobotics.frc2022.Superstructure;
import org.ghrobotics.frc2022.commands.DriveTrajectory;
import org.ghrobotics.frc2022.subsystems.Drivetrain;

public class RTFenderHigh3Ball extends SequentialCommandGroup {
  // Start at right tarmac up against fender; score 3 balls.
  public RTFenderHigh3Ball(RobotState robot_state, Drivetrain drivetrain,
                           Superstructure superstructure) {
    // Create routine.
    addCommands(
        new WaitCommand(1.0),

        // Reset odometry.
        new InstantCommand(() -> robot_state.resetPosition(
            AutoPlanner.kRTarmacFenderWallToBottomCargo.getInitialPose())),

        // Follow path to bottom ball while intaking.
        new ParallelRaceGroup(
            new DriveTrajectory(drivetrain, robot_state,
                AutoPlanner.kRTarmacFenderWallToBottomCargo),
            superstructure.trackGoalWithTurret(),
            superstructure.trackGoalWithHood(),
            superstructure.intake()
        ),

        // Score 2 balls (for 2.5 sec).
        superstructure.scoreHighGoal().withTimeout(3.5),

        // Pick up 3rd ball.
        new ParallelRaceGroup(
            new SequentialCommandGroup(
                new DriveTrajectory(drivetrain, robot_state,
                    AutoPlanner.kBottomCargoToIntermediateA),
                new DriveTrajectory(drivetrain, robot_state,
                    AutoPlanner.kIntermediateAToMiddleCargo)
            ),
            superstructure.trackGoalWithTurret(),
            superstructure.trackGoalWithHood(),
            superstructure.intake()
        ),

        // Score 1 ball (for 1.5 sec).
        superstructure.scoreHighGoal().withTimeout(3.5)
    );
  }
}
