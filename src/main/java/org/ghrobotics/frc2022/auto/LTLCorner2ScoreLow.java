package org.ghrobotics.frc2022.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.ghrobotics.frc2022.RobotState;
import org.ghrobotics.frc2022.Superstructure;
import org.ghrobotics.frc2022.commands.DriveTrajectory;
import org.ghrobotics.frc2022.subsystems.Drivetrain;

public class LTLCorner2ScoreLow extends SequentialCommandGroup {
    public LTLCorner2ScoreLow(RobotState robot_state, Drivetrain drivetrain, Superstructure superstructure) {
        addCommands(
                new InstantCommand(() -> robot_state.resetPosition(AutoPlanner.kLTarmacMLCornerToTopCargo.getInitialPose())),

                new ParallelRaceGroup(
                        new DriveTrajectory(drivetrain, robot_state, AutoPlanner.kLTarmacMLCornerToTopCargo),
                        superstructure.intake()
                ),

                new DriveTrajectory(drivetrain, robot_state, AutoPlanner.kTopCargoToLTarmacFender),
                superstructure.scoreHighGoalFender().withTimeout(3.5)
        );
    }
}
