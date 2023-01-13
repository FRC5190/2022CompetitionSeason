package org.ghrobotics.frc2022.planners;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.button.Button;
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
 * in the "superstructure" (turret, shooter, hood, feeder, intake).
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

  // Hints
  private double shooter_hint_ = 0;
  private double hood_hint_ = 0;

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

  // Misc Values
  double distance_ = 0;
  double turret_to_goal_distance = 0;
  double turret_to_goal_angle;
  double adjusted_distance = 0;
  
  // Next Cargo
  private Cargo next_up_cargo_ = Cargo.NONE;
  private double sensor_time_ = 0;

  /**
   * Constructs an instance of the superstructure planner.
   *
   * @param robot_state Reference to robot state.
   * @param turret      Reference to the turret subsystem.
   * @param shooter     Reference to the shooter subsystem.
   * @param hood        Reference to the hood subsystem.
   * @param feeder      Reference to the feeder subsystem.
   * @param intake      Reference to the intake subsystem.
   */
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

    MissionControl.addDouble("superstructure/turret_to_goal_distance", () -> distance_);

    MissionControl.addBoolean("superstructure/turret_at_goal", turret_::atGoal);
    MissionControl.addBoolean("superstructure/shooter_at_goal", shooter_::atGoal);
    MissionControl.addBoolean("superstructure/hood_at_goal", hood_::atGoal);

    MissionControl.addString("superstructure/turret_state", () -> turret_state_.name());
    MissionControl.addString("superstructure/shooter_state", () -> shooter_state_.name());
    MissionControl.addString("superstructure/hood_state", () -> hood_state_.name());
    MissionControl.addString("superstructure/feeder_state", () -> feeder_state_.name());
    MissionControl.addString("superstructure/intake_state", () -> intake_state_.name());

    MissionControl.addString("superstructure/next_up_cargo", () -> next_up_cargo_.name());

    // Add button to tune.
    new Button(() -> MissionControl.getBoolean(Constants.kStartTuneKey))
        .whenPressed(this::setTuning)
        .whenReleased(this::setDefault);
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
    turret_to_goal_distance = turret_to_goal.getNorm();
    turret_to_goal_angle = Math.atan2(turret_to_goal.getY(), turret_to_goal.getX());

    // Set variable for telemetry.
    distance_ = turret_to_goal_distance;

    // Calculate potential shooter speed and hood angle from lookup table at this distance.
    double maybe_shooter_speed = hg_planner_.getShooterSpeed(turret_to_goal_distance);
    double maybe_hood_angle = hg_planner_.getHoodAngle(turret_to_goal_distance);

    // Get the speed of the ball on the xy plane.
    double ball_vxy = Shooter.Constants.kWheelRadius * maybe_shooter_speed *
        Math.sin(maybe_hood_angle);

    // Calculate time of flight assuming no air resistance.
    double t = turret_to_goal_distance / ball_vxy * Constants.kTOFAdjustment;

    // Calculate new distance to goal, assuming approximately same time of flight.
    adjusted_distance = Math.sqrt(
        Math.pow(turret_to_goal_distance, 2) + Math.pow(robot_speeds.vxMetersPerSecond, 2) -
            2 * turret_to_goal_distance * robot_speeds.vxMetersPerSecond * t *
                Math.cos(turret_to_goal_angle));
    
    //adjusted_distance = Math.sqrt(
    //    Math.pow(turret_to_goal_distance, 2) + 
    //    Math.pow(robot_speeds.vxMetersPerSecond*Math.cos(turret_to_goal_angle)*t,2));
    
    //adjusted_distance = turret_to_goal_distance;

    // Use adjusted distance to adjust shooter speed.
    maybe_shooter_speed = (adjusted_distance / t * Constants.kTOFAdjustment) /
        Math.sin(maybe_hood_angle) / Shooter.Constants.kWheelRadius;

    // Update next up cargo from feeder.
    if (feeder_.getUpperSensor()) {
      next_up_cargo_ = toCargo(robot_state_.getAlliance(), feeder_.getUpperSensorColor());
      sensor_time_ = Timer.getFPGATimestamp();
    } else if (Timer.getFPGATimestamp() - sensor_time_ > 1.5 && !feeder_.getUpperSensor()) {
      next_up_cargo_ = Cargo.NONE;
    }

    // Go through the state for each subsystem and assign references.
    switch (turret_state_) {
      case TRACK:
        // Calculate turret angle adjustment.
        turret_pos_ = Turret.constrainSetpoint(turret_to_goal_angle +
            Math.asin(robot_speeds.vxMetersPerSecond * t * Math.sin(turret_to_goal_angle) /
                adjusted_distance));

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
    
    //if (shooter_state_ == ShooterState.IDLE && adjusted_distance >= Constants.kMinShootingDist && adjusted_distance <= Constants.kMaxShootingDist){
    //  shooter_state_ = ShooterState.HIGH_GOAL;
    //}

    switch (shooter_state_) {
      case IDLE:
      case CLIMB:
        shooter_speed_ = 0;
        break;
      case TUNING:
        shooter_speed_ = MissionControl.getDouble(Constants.kShooterSpeedKey);
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
        //System.out.println("Inside High goal");
        shooter_speed_ = maybe_shooter_speed;
        //System.out.println(maybe_shooter_speed);
        break;
      case USE_HINT:
        shooter_speed_ = shooter_hint_;
    }

    switch (hood_state_) {
      case TRACK:
        hood_angle_ = Math.toRadians(30);
        break;
      case HIGH_GOAL:
        hood_angle_ = maybe_hood_angle;
        break;
      case STOW:
        hood_angle_ = Constants.kStowHoodAngle;
        break;
      case TUNING:
        hood_angle_ = MissionControl.getDouble(Constants.kHoodAngleKey);
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
      case USE_HINT:
        hood_angle_ = hood_hint_;
    }

    switch (feeder_state_) {
      case IDLE:
      case INDEX:
        if (!feeder_.getUpperSensor()) {
          feeder_floor_pct_ = Constants.kFeederIndexPct;
          feeder_wall_pct_ = Constants.kFeederIndexPct;
        } else {
          feeder_floor_pct_ = feeder_.getLowerSensor() ? 0 : Constants.kFeederIndexPct;
          feeder_wall_pct_ = 0;
        }
        break;
      case FEED:
        feeder_floor_pct_ = Constants.kFeederFeedPct;
        feeder_wall_pct_ = Constants.kFeederFeedPct;
        break;
      case CLIMB:
        feeder_floor_pct_ = 0;
        feeder_wall_pct_ = 0;
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
        //intake_pct_ = Constants.kIntakeFeedPct;
        intake_pct_ = Constants.kIdleIntakePct;
        break;
    }

    // Set references.
    turret_.setGoal(turret_pos_, turret_vel_);

    //System.out.println(shooter_speed_);

    if (shooter_state_ == ShooterState.CLIMB){
      //System.out.println("Inside First Statement");
      shooter_.setPercent(Constants.kClimbShooterPct);
    }
    //else if ((shooter_state_ == ShooterState.IDLE || shooter_state_ == ShooterState.HIGH_GOAL) && adjusted_distance >= Constants.kMinShootingDist && adjusted_distance <= Constants.kMaxShootingDist){
      //System.out.println("Inside Second Statement");
      //if (robot_speeds.vxMetersPerSecond > 0.5 && feeder_.getUpperSensor()) { 
        //feeder_wall_pct_ = 0;
      //}
      //else {
    //   feeder_wall_pct_ = Constants.kFeederIndexPct;
      //}
    //  shooter_.setVelocity(shooter_speed_);
    //}
    else if (shooter_state_ == ShooterState.HIGH_GOAL){
      shooter_.setVelocity(shooter_speed_);      
    }
    else if (shooter_state_ == ShooterState.IDLE) {
      //System.out.println("Inside Third Statement");
      shooter_.setPercent(Constants.kIdleShooterPct);
    }
    //else if (adjusted_distance > Constants.kMaxShootingDist) {
    //  if (feeder_.getUpperSensor()) feeder_wall_pct_ = 0;
    //  shooter_.setVelocity(shooter_speed_);
      //shooter_state_ = ShooterState.IDLE;
    //}
    else {
      shooter_.setVelocity(shooter_speed_);
    }

    hood_.setPosition(hood_angle_);
    feeder_.setFloorPercent(feeder_floor_pct_);
    feeder_.setWallPercent(feeder_wall_pct_);
    intake_.setPercent(intake_pct_);
    intake_.setPivot(intake_out_);

    // Handle automatic state machine transitions:
    // There are no transitions to handle if the turret isn't tracking because it doesn't make
    // any sense if we aren't tracking the target but feeding balls, etc.
    if (turret_state_ == TurretState.TRACK) {
      // If we are in shooting states (fender low goal, fender high goal, or high goal), make sure
      // all systems are at reference before changing feeder state. Before that, make sure that
      // hood is tracking.
      if (hood_state_ == HoodState.TRACK || hood_state_ == HoodState.HIGH_GOAL ||
          hood_state_ == HoodState.FENDER_LOW_GOAL || hood_state_ == HoodState.FENDER_HIGH_GOAL ||
          hood_state_ == HoodState.TUNING)
        if (shooter_state_ == ShooterState.FENDER_LOW_GOAL ||
            shooter_state_ == ShooterState.FENDER_HIGH_GOAL ||
            shooter_state_ == ShooterState.HIGH_GOAL ||
            shooter_state_ == ShooterState.TUNING)
          if (turret_.atGoal() && shooter_.atGoal() && hood_.atGoal())
            feeder_state_ = FeederState.FEED;

      // If the next ball up is one of the wrong color, we need to eject it.
      if (next_up_cargo_ == Cargo.OPPOSITE) {
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
  }

  /**
   * Sets the superstructure to its default state, where the turret is tracking the goal, the
   * shooter is spinning at the idle RPM, and the hood is tracking the goal.
   */
  public void setDefault() {
    turret_state_ = TurretState.TRACK;
    shooter_state_ = ShooterState.IDLE;
    hood_state_ = HoodState.TRACK;
    feeder_state_ = FeederState.IDLE;
    intake_state_ = IntakeState.IDLE;
  }

  /**
   * Sets the superstructure to its climb state, with all systems off and stowed.
   */
  public void setClimb() {
    turret_state_ = TurretState.CLIMB;
    shooter_state_ = ShooterState.CLIMB;
    hood_state_ = HoodState.STOW;
    feeder_state_ = FeederState.CLIMB;
    intake_state_ = IntakeState.IDLE;
  }

  /**
   * Scores cargo into the low goal, assuming that the robot is at the fender.
   */
  public void setFenderLowGoal() {
    turret_state_ = TurretState.FENDER_SHOT;
    shooter_state_ = ShooterState.FENDER_LOW_GOAL;
    hood_state_ = HoodState.FENDER_LOW_GOAL;

    if (intake_state_ == IntakeState.IDLE)
      intake_state_ = IntakeState.FEED;
  }

  /**
   * Scores cargo into the high goal, assuming that the robot is at the fender.
   */
  public void setFenderHighGoal() {
    turret_state_ = TurretState.FENDER_SHOT;
    shooter_state_ = ShooterState.FENDER_HIGH_GOAL;
    hood_state_ = HoodState.FENDER_HIGH_GOAL;

    if (intake_state_ == IntakeState.IDLE)
      intake_state_ = IntakeState.FEED;
  }

  /**
   * Scores cargo into the high goal, using robot pose data to determine aiming and shooting
   * parameters.
   */
  public void setHighGoal() {
    turret_state_ = TurretState.TRACK;
    shooter_state_ = ShooterState.HIGH_GOAL;
    hood_state_ = HoodState.HIGH_GOAL;
    feeder_state_ = FeederState.FEED;

    if (intake_state_ == IntakeState.IDLE)
      intake_state_ = IntakeState.FEED;
  }

  /**
   * Cancels the scoring process.
   */
  public void cancelScoring() {
    turret_state_ = TurretState.TRACK;
    shooter_state_ = ShooterState.IDLE;
    hood_state_ = HoodState.TRACK;
    feeder_state_ = FeederState.IDLE;

    //if (intake_state_ == IntakeState.FEED)
      intake_state_ = IntakeState.IDLE;
  }

  /**
   * Uses the Mission Control dashboard to tune superstructure values.
   */
  public void setTuning() {
    turret_state_ = TurretState.TRACK;
    shooter_state_ = ShooterState.TUNING;
    hood_state_ = HoodState.TUNING;

    if (intake_state_ == IntakeState.IDLE)
      intake_state_ = IntakeState.FEED;
  }

  /**
   * Toggles the hood stow feature.
   */
  public void toggleHoodStow() {
    hood_state_ = hood_state_ == HoodState.STOW ? HoodState.TRACK : HoodState.STOW;
  }

  /**
   * Intakes cargo through the intake and indexes balls if necessary.
   */
  public void setIntake() {
    intake_state_ = IntakeState.INTAKE;
    if (feeder_state_ == FeederState.IDLE)
      feeder_state_ = FeederState.INDEX;
  }

  /**
   * Cancels the intaking process.
   */
  public void cancelIntake() {
    intake_state_ = IntakeState.IDLE;
    if (feeder_state_ != FeederState.FEED)
      feeder_state_ = FeederState.IDLE;
  }

  /**
   * Sets the shooter rpm and hood angle to the given hint (only used in auto) to speed up spin time.
   * @param pose The pose that the shot will be taken from.
   */
  public void setHint(Pose2d pose) {
    // Calculate distance to goal.
    double distance = pose.getTranslation().getDistance(Arena.kGoal);

    // Set hints.
    shooter_hint_ = hg_planner_.getShooterSpeed(distance);
    hood_hint_ = hg_planner_.getHoodAngle(distance);

    // Set states.
    shooter_state_ = ShooterState.USE_HINT;
    hood_state_ = HoodState.USE_HINT;
  }

  /**
   * Returns the current state of the turret.
   *
   * @return The current state of the turret.
   */
  public TurretState getTurretState() {
    return turret_state_;
  }

  /**
   * Returns the current state of the shooter.
   *
   * @return The current state of the shooter.
   */
  public ShooterState getShooterState() {
    return shooter_state_;
  }

  /**
   * Returns the current state of the hood.
   *
   * @return The current state of the hood.
   */
  public HoodState getHoodState() {
    return hood_state_;
  }

  /**
   * Returns the current state of the feeder.
   *
   * @return The current state of the feeder.
   */
  public FeederState getFeederState() {
    return feeder_state_;
  }

  /**
   * Returns the current state of the intake.
   *
   * @return The current state of the intake.
   */
  public IntakeState getIntakeState() {
    return intake_state_;
  }

  public Double getAdjustedDistance() {
    return adjusted_distance;
  }

  public Double getTurretToGoalDistance() {
    return turret_to_goal_distance;
  }

  public Double getTurretToGoalAngle() {
    return turret_to_goal_angle;
  }

  /**
   * Converts a color reading and current alliance to a cargo.
   *
   * @param alliance The alliance that the robot is on.
   * @param color    The color as reported by the sensor.
   * @return The type of cargo.
   */
  private static Cargo toCargo(DriverStation.Alliance alliance, Color color) {
    // If we have an invalid alliance for some reason, just assume that it's our own cargo.
    if (alliance == DriverStation.Alliance.Invalid)
      return Cargo.OWN;

    boolean red = color.red > 0.25;
    if (alliance == DriverStation.Alliance.Red)
      return red ? Cargo.OWN : Cargo.OPPOSITE;

    return red ? Cargo.OPPOSITE : Cargo.OWN;
  }

  public enum TurretState {
    TRACK,
    FENDER_SHOT,
    CLIMB
  }

  public enum ShooterState {
    IDLE,
    CLIMB,
    TUNING,
    EJECT,
    FENDER_LOW_GOAL,
    FENDER_HIGH_GOAL,
    HIGH_GOAL,
    USE_HINT,
  }

  public enum HoodState {
    TRACK,
    STOW,
    TUNING,
    EJECT,
    FENDER_LOW_GOAL,
    FENDER_HIGH_GOAL,
    HIGH_GOAL,
    USE_HINT
  }

  public enum FeederState {
    IDLE,
    INDEX,
    FEED,
    CLIMB
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
    public static final double kTOFAdjustment = 1.2;

    // Tuning
    public static final String kShooterSpeedKey = "superstructure/tuning_shooter_speed";
    public static final String kHoodAngleKey = "superstructure/tuning_hood_angle";
    public static final String kStartTuneKey = "superstructure/start_tune";

    // Climb
    public static final double kClimbTurretAngle = 3 * Math.PI / 2;
    public static final double kClimbShooterPct = 0;

    // Idle
    public static final double kIdleShooterPct = 0;
    public static final double kIdleIntakePct = 0;
    public static final double kIdleFeederPct = 0;

    // Stow
    public static final double kStowHoodAngle = Hood.Constants.kMinAngle;

    // Eject
    public static final double kEjectShooterRPM = 1000;
    public static final double kEjectHoodAngle = Math.toRadians(20);

    // Fender Low Goal
    public static final double kFenderLowGoalShooterRPM = 800;
    public static final double kFenderLowGoalHoodAngle = Math.toRadians(53);

    // Fender High Goal
    public static final double kFenderHighGoalShooterRPM = 2600;
    public static final double kFenderHighGoalHoodAngle = Math.toRadians(13);

    // Intake
    public static final double kIntakeIntakePct = 1.0;

    // Feed
    public static final double kIntakeFeedPct = 0.1;
    public static final double kFeederFeedPct = 0.4;

    // Index
    public static final double kFeederIndexPct = 0.3;

    //Shooting Range
    public static final double kMinShootingDist = 2.0;
    public static final double kMaxShootingDist = 4.0;

    //Hood
    public static final double kHoodAngle = 25;
  }
}
