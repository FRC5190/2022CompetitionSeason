package org.ghrobotics.frc2022.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.ghrobotics.frc2022.RobotState;
import org.ghrobotics.frc2022.commands.DriveTrajectory;
import org.ghrobotics.frc2022.planners.SuperstructurePlanner;
import org.ghrobotics.frc2022.subsystems.Drivetrain;

public class HighRight3 extends SequentialCommandGroup {
  /**
   * Shoots 3 balls into the high goal.
   *
   * @param robot_state    Reference to global robot state instance.
   * @param drivetrain     Reference to drivetrain subsystem.
   * @param superstructure Reference to superstructure planner.
   */
  public HighRight3(RobotState robot_state, Drivetrain drivetrain,
                    SuperstructurePlanner superstructure) {
    addCommands(
        // Reset odometry.
        new InstantCommand(
            () -> robot_state.resetPosition(AutoConfig.kRightStartToBottomCargo.getInitialPose())),

        // Intake cargo.
        new InstantCommand(superstructure::setIntake),

        // Set shooter rpm and hood angle hint.
        new InstantCommand(
            () -> superstructure.setHint(AutoConfig.kBottomCargoToIntermediateA.getInitialPose())),

        // Drive and pick up bottom cargo.
        new DriveTrajectory(drivetrain, robot_state, AutoConfig.kRightStartToBottomCargo),

        // Score for 2.5 sec
        new InstantCommand(superstructure::setHighGoal),
        new WaitCommand(2.5),
        new InstantCommand(superstructure::cancelScoring),

        // Set shooter rpm and hood angle hint.
        new InstantCommand(
            () -> superstructure.setHint(AutoConfig.kMiddleCargoToHP.getInitialPose())),

        // Pick up 3rd ball.
        new DriveTrajectory(drivetrain, robot_state, AutoConfig.kBottomCargoToIntermediateA),
        new DriveTrajectory(drivetrain, robot_state, AutoConfig.kIntermediateAToMiddleCargo),

        // Score for 2 sec
        new InstantCommand(superstructure::setHighGoal),
        new WaitCommand(2),
        new InstantCommand(superstructure::cancelScoring)
    );
  }
}
