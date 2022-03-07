package org.ghrobotics.frc2022.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.math.trajectory.constraint.EllipticalRegionConstraint;
import edu.wpi.first.math.trajectory.constraint.MaxVelocityConstraint;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.math.util.Units;
import java.util.List;

/**
 * This class handles the planning of all autonomous modes. It contains waypoints, regions, and
 * trajectories required to execute all auto modes.
 * <p>
 * Note that ALL measurements are from the Onshape Field CAD.
 */
public class AutoPlanner {
  // Waypoints
  public static final Translation2d kLTarmacFenderWall = createWaypoint(291.90, 174.57);
  public static final Translation2d kRTarmacFenderWall = createWaypoint(311.40, 130.62);

  public static final Translation2d kBottomCargo = createWaypoint(292.94, 12.25);
  public static final Translation2d kMiddleCargo = createWaypoint(193.83, 76.36);
  public static final Translation2d kTopCargo = createWaypoint(189.87, 239.30);
  public static final Translation2d kHPCargo = createWaypoint(38.12, 40.52);

  // Regions
  public static final CircularRegion kBottomCargoRegion = new CircularRegion(kBottomCargo, 1);
  public static final CircularRegion kMiddleCargoRegion = new CircularRegion(kMiddleCargo, 1);
  public static final CircularRegion kTopCargoRegion = new CircularRegion(kTopCargo, 1);
  public static final CircularRegion kHPCargoRegion = new CircularRegion(kHPCargo, 1);

  // Robot Dimensions
  public static final double kBumperThickness = 0.102;
  public static final double kDrivetrainLength = 0.38;
  public static final double kIntakeLength = 0.33;

  public static final Transform2d kRobotEdgeToCenter = new Transform2d(
      new Translation2d(-kBumperThickness - kDrivetrainLength / 2, 0), new Rotation2d());
  public static final Transform2d kIntakeToCenter = new Transform2d(
      new Translation2d(-kIntakeLength - kDrivetrainLength / 2, 0), new Rotation2d());

  // Trajectory Constraints
  public static final double kMaxVelocity = 3.5;
  public static final double kMaxCargoRegionVelocity = 0.5;
  public static final double kMaxAcceleration = 2.5;
  public static final double kMaxCentripetalAcceleration = 1.5;

  public static final MaxVelocityConstraint kRegionMaxVelocityConstraint =
      new MaxVelocityConstraint(kMaxCargoRegionVelocity);

  public static final TrajectoryConfig kConfig =
      new TrajectoryConfig(kMaxVelocity, kMaxAcceleration)
          .addConstraint(new CentripetalAccelerationConstraint(kMaxCentripetalAcceleration))
          .addConstraint(kBottomCargoRegion.createConstraint(kRegionMaxVelocityConstraint))
          .addConstraint(kMiddleCargoRegion.createConstraint(kRegionMaxVelocityConstraint))
          .addConstraint(kTopCargoRegion.createConstraint(kRegionMaxVelocityConstraint))
          .addConstraint(kHPCargoRegion.createConstraint(kRegionMaxVelocityConstraint));

  // Trajectories
  public static final Trajectory kRTarmacFenderWallToBottomCargo = createTrajectory(
      new Pose2d(kRTarmacFenderWall, Rotation2d.fromDegrees(201)).transformBy(kRobotEdgeToCenter),
      new Pose2d(kBottomCargo, Rotation2d.fromDegrees(270)).transformBy(kIntakeToCenter), false);

  public static final Trajectory kBottomCargoToIntermediateA = createTrajectory(
      new Pose2d(kBottomCargo, Rotation2d.fromDegrees(270)).transformBy(kIntakeToCenter),
      new Pose2d(kBottomCargo, Rotation2d.fromDegrees(270))
          .transformBy(new Transform2d(new Translation2d(-1, 0.3), Rotation2d.fromDegrees(-90))),
      true);

  public static final Trajectory kIntermediateAToMiddleCargoToHPCargo = createTrajectory(
      new Pose2d(kBottomCargo, Rotation2d.fromDegrees(270))
          .transformBy(new Transform2d(new Translation2d(-1, 0.3), Rotation2d.fromDegrees(-90))),
      List.of(kMiddleCargo),
      new Pose2d(kHPCargo, Rotation2d.fromDegrees(225)), false);

  public static final Trajectory kHPCargoToMiddleCargo = createTrajectory(
      new Pose2d(kHPCargo, Rotation2d.fromDegrees(225)),
      new Pose2d(kMiddleCargo, Rotation2d.fromDegrees(190)), true);


  /**
   * Creates a waypoint from the provided measurements (in inches).
   *
   * @param x The x position in inches.
   * @param y The y position in inches.
   * @return The waypoint (in SI units) from the provided measurements.
   */
  private static Translation2d createWaypoint(double x, double y) {
    return new Translation2d(Units.inchesToMeters(x), Units.inchesToMeters(y));
  }

  /**
   * Creates a trajectory from the provided waypoints.
   *
   * @param start_waypoint     The starting pose.
   * @param interior_waypoints The interior waypoints.
   * @param end_waypoint       The ending pose.
   * @param reversed           Whether the robot should travel in reverse.
   * @return Trajectory from the provided waypoints.
   */
  private static Trajectory createTrajectory(Pose2d start_waypoint,
                                             List<Translation2d> interior_waypoints,
                                             Pose2d end_waypoint, boolean reversed) {
    kConfig.setReversed(reversed);
    return TrajectoryGenerator.generateTrajectory(
        start_waypoint, interior_waypoints, end_waypoint, kConfig);
  }

  private static Trajectory createTrajectory(Pose2d start_waypoint, Pose2d end_waypoint,
                                             boolean reversed) {
    return createTrajectory(start_waypoint, List.of(), end_waypoint, reversed);
  }

  private static class CircularRegion {
    private final Translation2d center_;
    private final double radius_;

    public CircularRegion(Translation2d center, double radius) {
      center_ = center;
      radius_ = radius;
    }

    public boolean isPoseInRegion(Pose2d pose) {
      return Math.hypot(pose.getX() - center_.getX(), pose.getY() - center_.getY()) < radius_;
    }

    public EllipticalRegionConstraint createConstraint(TrajectoryConstraint constraint) {
      return new EllipticalRegionConstraint(center_, radius_, radius_, new Rotation2d(),
          constraint);
    }
  }
}
