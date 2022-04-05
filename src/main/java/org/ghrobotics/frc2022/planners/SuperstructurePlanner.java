package org.ghrobotics.frc2022.planners;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import org.ghrobotics.frc2022.Arena;
import org.ghrobotics.frc2022.RobotState;
import org.ghrobotics.frc2022.subsystems.Feeder;
import org.ghrobotics.frc2022.subsystems.Hood;
import org.ghrobotics.frc2022.subsystems.Intake;
import org.ghrobotics.frc2022.subsystems.Shooter;
import org.ghrobotics.frc2022.subsystems.Turret;
import org.ghrobotics.lib.telemetry.MissionControl;

/**
 * This class is responsible for planning out the motor and pneumatics outputs for all subsystems
 * in the "superstructure" (turret, shooter, hood, intake).
 */
public class SuperstructurePlanner {
  // Robot State
  private final RobotState robot_state_;

  // Subsystems
  private final Turret turret_;
  private final Shooter shooter_;
  private final Hood hood_;
  private final Feeder feeder_;
  private final Intake intake_;

  // Subsystem States
  private TurretState turret_state_ = TurretState.TRACK;
  private ShooterState shooter_state_ = ShooterState.IDLE;
  private HoodState hood_state_ = HoodState.TRACK;
  private FeederState feeder_state_ = FeederState.IDLE;
  private IntakeState intake_state_ = IntakeState.IDLE;

  // Subsystem References
  private double turret_pos_;
  private double turret_vel_;
  private double shooter_speed_;
  private double hood_angle_;
  private double intake_pct_;
  private double feeder_floor_pct_;
  private double feeder_wall_pct_;
  private boolean intake_out_;

  // High Goal Planner
  private final HighGoalPlanner hg_planner_;

  // Next Cargo
  private Cargo next_up_cargo_ = Cargo.NONE;

  public SuperstructurePlanner(RobotState robot_state, Turret turret, Shooter shooter, Hood hood,
                               Feeder feeder, Intake intake) {
    // Assign robot state and subsystems.
    robot_state_ = robot_state;
    turret_ = turret;
    shooter_ = shooter;
    hood_ = hood;
    feeder_ = feeder;
    intake_ = intake;

    // Create high goal planner.
    hg_planner_ = new HighGoalPlanner(Constants.kFenderHighGoalShooterRPM,
        Constants.kFenderHighGoalHoodAngle);

    // Add telemetry.
    MissionControl.addDouble("superstructure/turret_position_setpoint", () -> turret_pos_);
    MissionControl.addDouble("superstructure/turret_velocity_setpoint", () -> turret_vel_);
    MissionControl.addDouble("superstructure/shooter_speed_setpoint", () -> shooter_speed_);
    MissionControl.addDouble("superstructure/hood_angle_setpoint", () -> hood_angle_);

    MissionControl.addBoolean("superstructure/turret_at_goal", turret_::atGoal);
    MissionControl.addBoolean("superstructure/shooter_at_goal", shooter_::atGoal);
    MissionControl.addBoolean("superstructure/hood_at_goal", hood_::atGoal);

    MissionControl.addString("superstructure/turret_state", () -> turret_state_.name());
    MissionControl.addString("superstructure/shooter_state", () -> shooter_state_.name());
    MissionControl.addString("superstructure/hood_state", () -> hood_state_.name());
    MissionControl.addString("superstructure/feeder_state", () -> feeder_state_.name());
    MissionControl.addString("superstructure/intake_state", () -> intake_state_.name());
  }

  /**
   * Updates the superstructure state machine with new information, either from an autonomous
   * command or teleop control.
   */
  public void update() {
    // Get the latest robot pose and speeds.
    Pose2d robot_pose = robot_state_.getRobotPose();
    ChassisSpeeds robot_speeds = robot_state_.getRobotSpeeds();

    // Calculate turret pose.
    Pose2d turret_pose = robot_pose.plus(
        new Transform2d(new Translation2d(Constants.kRobotToTurretDistance, 0), new Rotation2d()));

    // Calculate translation to goal.
    Translation2d turret_to_goal = new Pose2d(Arena.kGoal, new Rotation2d())
        .relativeTo(turret_pose).getTranslation();

    // Calculate distance and angle to goal.
    double turret_to_goal_distance = turret_to_goal.getNorm();
    double turret_to_goal_angle = Math.atan2(turret_to_goal.getY(), turret_to_goal.getX());

    // Calculate potential shooter speed and hood angle from lookup table at this distance.
    double maybe_shooter_speed = hg_planner_.getShooterSpeed(turret_to_goal_distance);
    double maybe_hood_angle = hg_planner_.getHoodAngle(turret_to_goal_distance);

    // Get the speed of the ball on the xy plane.
    double ball_vxy = Shooter.Constants.kWheelRadius * maybe_shooter_speed *
        Math.sin(maybe_hood_angle);

    // Calculate time of flight assuming no air resistance.
    double t = turret_to_goal_distance / ball_vxy * Constants.kTOFAdjustment;

    // Calculate new distance to goal, assuming approximately same time of flight.
    double adjusted_distance = Math.sqrt(
        Math.pow(turret_to_goal_distance, 2) + Math.pow(robot_speeds.vxMetersPerSecond, 2) -
            2 * turret_to_goal_distance * robot_speeds.vxMetersPerSecond * t *
                Math.cos(turret_to_goal_angle));

    // Use adjusted distance to adjust shooter speed.
    maybe_shooter_speed = (adjusted_distance / t) / Math.sin(maybe_hood_angle) /
        Shooter.Constants.kWheelRadius;

    // Go through the state for each subsystem and assign references.
    switch (turret_state_) {
      case TRACK:
        // Calculate turret angle adjustment.
        turret_pos_ = turret_to_goal_angle +
            Math.asin(robot_speeds.vxMetersPerSecond * t * Math.sin(turret_to_goal_angle) /
                adjusted_distance);

        // Calculate angular velocity to track goal.
        turret_vel_ = -robot_speeds.omegaRadiansPerSecond - robot_speeds.vxMetersPerSecond *
            Math.sin(turret_pos_) / turret_to_goal_distance;
        break;
      case FENDER_SHOT:
        turret_pos_ = Math.PI;
        turret_vel_ = 0;
        break;
      case CLIMB:
        turret_pos_ = Constants.kClimbTurretAngle;
        turret_vel_ = 0;
        break;
    }

    switch (shooter_state_) {
      case IDLE:
        shooter_speed_ = Units.rotationsPerMinuteToRadiansPerSecond(Constants.kIdleShooterRPM);
        break;
      case CLIMB:
        shooter_speed_ = Units.rotationsPerMinuteToRadiansPerSecond(Constants.kClimbShooterPct);
        break;
      case EJECT:
        shooter_speed_ = Units.rotationsPerMinuteToRadiansPerSecond(Constants.kEjectShooterRPM);
        break;
      case FENDER_LOW_GOAL:
        shooter_speed_ = Units.rotationsPerMinuteToRadiansPerSecond(
            Constants.kFenderLowGoalShooterRPM);
        break;
      case FENDER_HIGH_GOAL:
        shooter_speed_ = Units.rotationsPerMinuteToRadiansPerSecond(
            Constants.kFenderHighGoalShooterRPM);
        break;
      case HIGH_GOAL:
        shooter_speed_ = maybe_shooter_speed;
        break;
    }

    switch (hood_state_) {
      case TRACK:
        break;
      case STOW:
        hood_angle_ = Constants.kStowHoodAngle;
        break;
      case EJECT:
        hood_angle_ = Constants.kEjectHoodAngle;
        break;
      case FENDER_LOW_GOAL:
        hood_angle_ = Constants.kFenderLowGoalHoodAngle;
        break;
      case FENDER_HIGH_GOAL:
        hood_angle_ = Constants.kFenderHighGoalHoodAngle;
        break;
      case HIGH_GOAL:
        hood_angle_ = maybe_hood_angle;
        break;
    }

    switch (feeder_state_) {
      case IDLE:
        feeder_floor_pct_ = Constants.kIdleFeederPct;
        feeder_wall_pct_ = Constants.kIdleFeederPct;
        break;
      case INDEX:
        break;
      case FEED:
        break;
    }

    switch (intake_state_) {
      case IDLE:
        intake_out_ = false;
        intake_pct_ = Constants.kIdleIntakePct;
        break;
      case INTAKE:
        intake_out_ = true;
        intake_pct_ = Constants.kIntakeIntakePct;
        break;
      case FEED:
        intake_out_ = false;
        intake_pct_ = Constants.kIntakeFeedPct;
        break;
    }

    // Handle automatic state machine transitions:
    // There are no transitions to handle if the turret isn't tracking because it doesn't make
    // any sense if we aren't tracking the target but feeding balls, etc.
    if (turret_state_ == TurretState.TRACK) {
      // If we are in shooting states (fender low goal, fender high goal, or high goal), make sure
      // all systems are at reference before changing feeder state. Before that, make sure that
      // hood is tracking.
      if (hood_state_ == HoodState.TRACK)
        if (shooter_state_ == ShooterState.FENDER_LOW_GOAL ||
            shooter_state_ == ShooterState.FENDER_HIGH_GOAL ||
            shooter_state_ == ShooterState.HIGH_GOAL)
          if (turret_.atGoal() && shooter_.atGoal() && hood_.atGoal())
            feeder_state_ = FeederState.FEED;

      // If the next ball up is one of the wrong color, we need to eject it. But make sure hood
      // is tracking before doing this. Otherwise, for example, if we are stowed and raise the
      // hood to eject, we might hit the L1 bar on the hangar.
      if (next_up_cargo_ == Cargo.OPPOSITE && hood_state_ == HoodState.TRACK) {
        shooter_state_ = ShooterState.EJECT;
        hood_state_ = HoodState.EJECT;
        feeder_state_ = FeederState.FEED;
      }

      // If we were previously ejecting and the ball no longer exists, stop.
      if (shooter_state_ == ShooterState.EJECT && hood_state_ == HoodState.EJECT &&
          next_up_cargo_ != Cargo.OPPOSITE) {
        shooter_state_ = ShooterState.IDLE;
        hood_state_ = HoodState.TRACK;
        feeder_state_ = FeederState.IDLE;
      }
    }

    // Set references.
    turret_.setGoal(turret_pos_, turret_vel_);
    shooter_.setVelocity(shooter_speed_);
    hood_.setPosition(hood_angle_);
    feeder_.setFloorPercent(feeder_floor_pct_);
    feeder_.setWallPercent(feeder_wall_pct_);
    intake_.setPercent(intake_pct_);
    intake_.setPivot(intake_out_);
  }

  /**
   * Scores cargo into the low goal from the fender.
   */
  public void scoreFenderLowGoal() {
    turret_state_ = TurretState.FENDER_SHOT;
    shooter_state_ = ShooterState.FENDER_LOW_GOAL;
    hood_state_ = HoodState.FENDER_LOW_GOAL;
  }

  /**
   * Scores cargo into the high goal from the fender.
   */
  public void scoreFenderHighGoal() {
    turret_state_ = TurretState.FENDER_SHOT;
    shooter_state_ = ShooterState.FENDER_HIGH_GOAL;
    hood_state_ = HoodState.FENDER_HIGH_GOAL;
  }

  /**
   * Scores cargo into the high goal.
   */
  public void scoreHighGoal() {
    turret_state_ = TurretState.TRACK;
    shooter_state_ = ShooterState.HIGH_GOAL;
    hood_state_ = HoodState.TRACK;
  }

  /**
   * Sets the superstructure into its climb state.
   */
  public void climb() {
    turret_state_ = TurretState.CLIMB;
    shooter_state_ = ShooterState.CLIMB;
    hood_state_ = HoodState.STOW;
    intake_state_ = IntakeState.IDLE;
    feeder_state_ = FeederState.IDLE;
  }

  /**
   * Sets the superstructure into its default state.
   */
  public void defaultState() {
    turret_state_ = TurretState.TRACK;
    shooter_state_ = ShooterState.IDLE;
    hood_state_ = HoodState.TRACK;
    intake_state_ = IntakeState.IDLE;
    feeder_state_ = FeederState.IDLE;
  }

  public enum TurretState {
    TRACK,
    FENDER_SHOT,
    CLIMB
  }

  public enum ShooterState {
    IDLE,
    CLIMB,
    EJECT,
    FENDER_LOW_GOAL,
    FENDER_HIGH_GOAL,
    HIGH_GOAL
  }

  public enum HoodState {
    TRACK,
    STOW,
    EJECT,
    FENDER_LOW_GOAL,
    FENDER_HIGH_GOAL,
    HIGH_GOAL
  }

  public enum FeederState {
    IDLE,
    INDEX,
    FEED
  }

  public enum IntakeState {
    IDLE,
    INTAKE,
    FEED
  }

  public enum Cargo {
    OWN, OPPOSITE, NONE
  }

  public static class Constants {
    // Measurements
    public static final double kRobotToTurretDistance = 0.2;

    // Shooting While Moving
    public static final double kTOFAdjustment = 1.0;

    // Climb
    public static final double kClimbTurretAngle = Math.PI / 2;
    public static final double kClimbShooterPct = 0;

    // Idle
    public static final double kIdleShooterRPM = 500;
    public static final double kIdleIntakePct = 0;
    public static final double kIdleFeederPct = 0;

    // Stow
    public static final double kStowHoodAngle = Hood.Constants.kMinAngle;

    // Eject
    public static final double kEjectShooterRPM = 750;
    public static final double kEjectHoodAngle = Math.toRadians(9);

    // Fender Low Goal
    public static final double kFenderLowGoalShooterRPM = 1200;
    public static final double kFenderLowGoalHoodAngle = Math.toRadians(42);

    // Fender High Goal
    public static final double kFenderHighGoalShooterRPM = 2600;
    public static final double kFenderHighGoalHoodAngle = Math.toRadians(10);

    // Intake
    public static final double kIntakeIntakePct = 1.0;

    // Feed
    public static final double kIntakeFeedPct = 0.5;
  }
}
