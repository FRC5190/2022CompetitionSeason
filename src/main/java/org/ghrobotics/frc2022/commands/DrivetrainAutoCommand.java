package org.ghrobotics.frc2022.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.ghrobotics.frc2022.RobotState;
import org.ghrobotics.frc2022.subsystems.Drivetrain;

public class DrivetrainAutoCommand extends CommandBase {
  // Subsystem Reference
  private final Drivetrain drivetrain_;

  // Robot State
  private final RobotState robot_state_;

  // Objects needed to track trajectory.
  private final Trajectory trajectory_;
  private final Timer timer_;

  /**
   * Follows a trajectory.
   *
   * @param drivetrain  A reference to the Drivetrain subsystem.
   * @param robot_state A reference to the global robot state.
   * @param trajectory  The trajectory to follow.
   */
  public DrivetrainAutoCommand(Drivetrain drivetrain, RobotState robot_state,
                               Trajectory trajectory) {
    // Assign local variables.
    drivetrain_ = drivetrain;
    robot_state_ = robot_state;
    trajectory_ = trajectory;

    // Create timer object.
    timer_ = new Timer();

    // Set subsystem requirements.
    addRequirements(drivetrain_);
  }

  @Override
  public void initialize() {
    // Start the timer.
    timer_.start();
  }

  @Override
  public void execute() {
    // Get time elapsed.
    double t = timer_.get();

    // Get desired state at current time.
    Trajectory.State desired_state = trajectory_.sample(t);

    // Get robot pose at the current time.
    Pose2d current_state = robot_state_.getRobotPose();

    // Calculate chassis speeds to track desired state.
    ChassisSpeeds wanted_chassis_speeds = drivetrain_.getRamseteController().calculate(
        current_state, desired_state);

    // Convert to wheel speeds.
    DifferentialDriveWheelSpeeds wanted_wheel_speeds = drivetrain_.getKinematics().toWheelSpeeds(
        wanted_chassis_speeds);

    // Set wheel speeds on drivetrain.
    drivetrain_.setVelocity(wanted_wheel_speeds.leftMetersPerSecond,
        wanted_wheel_speeds.rightMetersPerSecond);
  }

  @Override
  public void end(boolean interrupted) {
    // If we are interrupted, something went wrong; set speeds to 0.
    if (interrupted)
      drivetrain_.setPercent(0, 0);

    // Stop and reset the timer.
    timer_.stop();
    timer_.reset();
  }

  @Override
  public boolean isFinished() {
    // We are done when the elapsed time is greater than the trajectory's total time.
    return timer_.get() > trajectory_.getTotalTimeSeconds();
  }
}
