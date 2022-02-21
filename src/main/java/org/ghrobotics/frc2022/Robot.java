// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.ghrobotics.frc2022;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;
import org.ghrobotics.frc2022.commands.DrivetrainAutoCommand;
import org.ghrobotics.frc2022.commands.DrivetrainTeleopCommand;
import org.ghrobotics.frc2022.subsystems.Drivetrain;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  // Create our Xbox Controller
  private final XboxController controller_ = new XboxController(0);

  // Create subsystems.
  private final Drivetrain drivetrain_ = new Drivetrain();

  @Override
  public void robotInit() {
    // Set default commands for subsystems.
    drivetrain_.setDefaultCommand(new DrivetrainTeleopCommand(drivetrain_, controller_));

  }

  @Override
  public void disabledInit() {}

  @Override
  public void autonomousInit() {
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d()),
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)), new Pose2d(3, 0, new Rotation2d()),
        new TrajectoryConfig(0.5, 0.5));
    drivetrain_.resetPosition(new Pose2d());
    new DrivetrainAutoCommand(drivetrain_, trajectory).schedule();
  }

  @Override
  public void teleopInit() {}

  @Override
  public void testInit() {}

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    NetworkTableInstance.getDefault().flush();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testPeriodic() {}
}
