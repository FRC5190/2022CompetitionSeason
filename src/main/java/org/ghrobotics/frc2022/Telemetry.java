package org.ghrobotics.frc2022;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.BooleanSupplier;
import org.ghrobotics.frc2022.subsystems.Climber;
import org.ghrobotics.frc2022.subsystems.Drivetrain;
import org.ghrobotics.frc2022.subsystems.Hood;
import org.ghrobotics.frc2022.subsystems.Intake;
import org.ghrobotics.frc2022.subsystems.Shooter;
import org.ghrobotics.frc2022.subsystems.Turret;
import org.ghrobotics.lib.telemetry.MissionControl;

public class Telemetry {
  // Shuffleboard Tab
  private final ShuffleboardTab tab_;

  // Robot State
  private final RobotState robot_state_;

  // Subsystems
  private final Drivetrain drivetrain_;
  private final Turret turret_;
  private final Shooter shooter_;
  private final Hood hood_;
  private final Intake intake_;
  private final Climber climber_;

  // Sendable Chooser
  private final SendableChooser<Command> auto_selector_;

  // Misc Data Sources
  private final BooleanSupplier climb_mode_;

  // Field
  private final Field2d field_;

  /**
   * Handles reporting of telemetry to the drivers / programmers, including autonomous mode
   * chooser via Shuffleboard.
   *
   * @param robot_state   Reference to robot state.
   * @param drivetrain    Reference to drivetrain subsystem.
   * @param turret        Reference to turret subsystem.
   * @param shooter       Reference to shooter subsystem.
   * @param hood          Reference to hood subsystem.
   * @param intake        Reference to intake subsystem.
   * @param climber       Reference to climber subsystem.
   * @param auto_selector Reference to autonomous mode selector.
   * @param climb_mode    Supplier for climb mode.
   */
  public Telemetry(RobotState robot_state, Drivetrain drivetrain, Turret turret, Shooter shooter,
                   Hood hood, Intake intake, Climber climber,
                   SendableChooser<Command> auto_selector, BooleanSupplier climb_mode) {
    // Create Shuffleboard tab to show all robot information.
    tab_ = Shuffleboard.getTab("Apex");

    // Assign robot state.
    robot_state_ = robot_state;

    // Assign subsystems.
    drivetrain_ = drivetrain;
    turret_ = turret;
    shooter_ = shooter;
    hood_ = hood;
    intake_ = intake;
    climber_ = climber;

    // Assign auto selector.
    auto_selector_ = auto_selector;

    // Assign misc data sources.
    climb_mode_ = climb_mode;

    // Initialize field and add it to SmartDashboard (we don't need to view this on the primary
    // Shuffleboard tab).
    field_ = new Field2d();
    SmartDashboard.putData("Field", field_);
    MissionControl.addSendable("field", field_);

    // Put autonomous mode selector on Shuffleboard.
    tab_.add("Autonomous Mode Selector", auto_selector_)
        .withSize(3, 2)
        .withPosition(0, 0);

    // Add primary robot state information.
    ShuffleboardLayout robot_state_layout = tab_.getLayout("Robot State", BuiltInLayouts.kGrid)
        .withSize(2, 2)
        .withPosition(3, 0);
    robot_state_layout.addNumber("Robot X (ft)",
            () -> Units.metersToFeet(robot_state_.getRobotPose().getX()))
        .withPosition(0, 0);
    robot_state_layout.addNumber("Robot Y (ft)",
            () -> Units.metersToFeet(robot_state_.getRobotPose().getY()))
        .withPosition(0, 1);
    robot_state_layout.addNumber("Robot Angle (deg)",
            () -> robot_state_.getRobotPose().getRotation().getDegrees())
        .withPosition(0, 2);

    // Add drivetrain information.
    ShuffleboardLayout drivetrain_layout = tab_.getLayout("Drivetrain", BuiltInLayouts.kGrid)
        .withSize(2, 2)
        .withPosition(5, 0);
    drivetrain_layout.addNumber("L Position (m)", drivetrain::getLeftPosition)
        .withPosition(0, 0);
    drivetrain_layout.addNumber("R Position (m)", drivetrain::getRightPosition)
        .withPosition(0, 1);
    drivetrain_layout.addNumber("L Velocity (mps)", drivetrain::getLeftVelocity)
        .withPosition(1, 0);
    drivetrain_layout.addNumber("R Velocity (mps)", drivetrain::getRightVelocity)
        .withPosition(1, 1);

    // Add scoring subsystem information (turret, shooter, hood).
    ShuffleboardLayout scoring_layout = tab_.getLayout("Scoring Subsystems", BuiltInLayouts.kGrid)
        .withSize(3, 2)
        .withPosition(0, 2);
    scoring_layout.addNumber("Turret Position (deg)", () -> Math.toDegrees(turret_.getPosition()))
        .withPosition(0, 0);
    scoring_layout.addNumber("Turret Velocity (dps)", () -> Math.toDegrees(turret_.getVelocity()))
        .withPosition(1, 0);
    scoring_layout.addNumber("Shooter Velocity (rpm)",
            () -> Units.radiansPerSecondToRotationsPerMinute(shooter_.getVelocity()))
        .withPosition(0, 1);
    scoring_layout.addNumber("Hood Position (deg)", () -> Math.toDegrees(hood_.getPosition()))
        .withPosition(1, 1);

    // Add climber information.
    ShuffleboardLayout climber_layout = tab_.getLayout("Climber", BuiltInLayouts.kGrid)
        .withSize(3, 2)
        .withPosition(3, 2);
    climber_layout.addNumber("L Position (in)",
            () -> Units.metersToInches(climber_.getLeftPosition()))
        .withPosition(0, 0);
    climber_layout.addNumber("R Position (in)",
            () -> Units.metersToInches(climber_.getRightPosition()))
        .withPosition(1, 0);
    climber_layout.addNumber("L Current (A)", climber::getLeftSupplyCurrent)
        .withPosition(0, 1);
    climber_layout.addNumber("R Current (A)", climber::getRightSupplyCurrent)
        .withPosition(1, 1);
    climber_layout.addBoolean("Climb Mode", climb_mode_)
        .withPosition(2, 0);
  }

  public void periodic() {
    // Get robot pose from state.
    Pose2d robot_pose = robot_state_.getRobotPose();

    // Update field view with robot pose.
    field_.setRobotPose(robot_pose);

    // Update field view with turret pose.
    field_.getObject("Turret").setPose(
        robot_pose.transformBy(
            new Transform2d(new Translation2d(), robot_state_.getTurretAngle())));
  }
}
