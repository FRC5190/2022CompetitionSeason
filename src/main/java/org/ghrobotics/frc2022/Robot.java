// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.ghrobotics.frc2022;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
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
    // Set our default command.
    drivetrain_.setDefaultCommand(
        new RunCommand(() -> {
          DifferentialDrive.WheelSpeeds speeds = DifferentialDrive.curvatureDriveIK(
              -controller_.getLeftY(), controller_.getLeftX(), controller_.getAButton()
          );
          drivetrain_.setVoltages(speeds.left * 1, speeds.right * 1);
        }, drivetrain_)
    );
  }

  @Override
  public void disabledInit() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void teleopInit() {}

  @Override
  public void testInit() {}

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
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
