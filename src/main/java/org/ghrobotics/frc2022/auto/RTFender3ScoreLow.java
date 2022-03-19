package org.ghrobotics.frc2022.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.ghrobotics.frc2022.RobotState;
import org.ghrobotics.frc2022.Superstructure;
import org.ghrobotics.frc2022.commands.DriveTrajectory;
import org.ghrobotics.frc2022.subsystems.Drivetrain;

public class RTFender3ScoreLow extends SequentialCommandGroup {
    public RTFender3ScoreLow(RobotState robot_state, Drivetrain drivetrain, Superstructure superstructure) {
        addCommands(
                new InstantCommand(() -> robot_state.resetPosition(AutoPlanner.kRTarmacFenderWallToBottomCargo.getInitialPose())),

                // Follow path to bottom ball while intaking.
                new ParallelRaceGroup(
                        new DriveTrajectory(drivetrain, robot_state,
                                AutoPlanner.kRTarmacFenderWallToBottomCargo),
                        superstructure.intake()
                ),

                new DriveTrajectory(drivetrain, robot_state, AutoPlanner.kBottomCargoToRTarmacFenderWall),
                superstructure.scoreLowGoalFender().withTimeout(3.0),

                new ParallelRaceGroup(
                        new DriveTrajectory(drivetrain, robot_state,
                                AutoPlanner.kRTarmacFenderWallToMiddleCargo),
                        superstructure.intake()
                ),

                new DriveTrajectory(drivetrain, robot_state, AutoPlanner.kMiddleCargoToRTarmacFenderWall),
                superstructure.scoreLowGoalFender().withTimeout(3.5)

        );

    }
}
