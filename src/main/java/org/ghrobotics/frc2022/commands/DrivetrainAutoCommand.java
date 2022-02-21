package org.ghrobotics.frc2022.commands;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.ghrobotics.frc2022.subsystems.Drivetrain;

public class DrivetrainAutoCommand extends CommandBase {
  private final Drivetrain drivetrain_;
  private final Trajectory trajectory_;
  private final RamseteController controller_;

  private final Timer timer_;

  public DrivetrainAutoCommand(Drivetrain drivetrain, Trajectory trajectory) {
    drivetrain_ = drivetrain;
    trajectory_ = trajectory;

    controller_ = new RamseteController();
    timer_ = new Timer();

    addRequirements(drivetrain_);
  }

  @Override
  public void initialize() {
    timer_.start();
  }

  @Override
  public void execute() {
    double t = timer_.get();
    Trajectory.State desired_state = trajectory_.sample(t);
    Pose2d current_state = drivetrain_.getPosition();
    ChassisSpeeds chassis_speeds = controller_.calculate(current_state, desired_state);
    DifferentialDriveWheelSpeeds wheel_speeds = drivetrain_.getKinematics().toWheelSpeeds(chassis_speeds);
    drivetrain_.setVelocity(wheel_speeds.leftMetersPerSecond, wheel_speeds.rightMetersPerSecond);
  }

  @Override
  public boolean isFinished() {
    return timer_.get() > trajectory_.getTotalTimeSeconds();
  }
}
