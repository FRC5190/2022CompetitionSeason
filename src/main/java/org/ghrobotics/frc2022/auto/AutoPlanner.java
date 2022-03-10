package org.ghrobotics.frc2022.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
  public static final Translation2d kBottomCargo = createWaypoint(292.94, 12.25);
  public static final Translation2d kMiddleCargo = createWaypoint(193.83, 76.36);
  public static final Translation2d kTopCargo = createWaypoint(189.87, 239.30);
  public static final Translation2d kHPCargo = createWaypoint(38.12, 40.52);

  // Regions
  public static final CircularRegion kBottomCargoRegion = new CircularRegion(kBottomCargo, 1);
  public static final CircularRegion kMiddleCargoRegion = new CircularRegion(kMiddleCargo, 1);
  public static final CircularRegion kTopCargoRegion = new CircularRegion(kTopCargo, 1);
  public static final CircularRegion kHPCargoRegion = new CircularRegion(kHPCargo, 1);

  // Trajectory Constraints
  public static final double kMaxVelocity = 2.5;
  public static final double kMaxCargoRegionVelocity = 0.5;
  public static final double kMaxAcceleration = 1.0;
  public static final double kMaxCentripetalAcceleration = .5;

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
      new Pose2d(7.773, 2.903, Rotation2d.fromDegrees(249)),
      new Pose2d(7.602, 0.830, Rotation2d.fromDegrees(270)),
      false);

  public static final Trajectory kBottomCargoToIntermediateA = createTrajectory(
      new Pose2d(7.602, 0.830, Rotation2d.fromDegrees(270)),
      new Pose2d(8.298, 1.256, Rotation2d.fromDegrees(182)),
      true);

  public static final Trajectory kIntermediateAToMiddleCargo = createTrajectory(
      new Pose2d(8.298, 1.256, Rotation2d.fromDegrees(182)),
      new Pose2d(5.387, 1.710, Rotation2d.fromDegrees(148)),
      false);

  public static final Trajectory kIntermediateAToMiddleCargoToHPCargo = createTrajectory(
      new Pose2d(8.298, 1.256, Rotation2d.fromDegrees(182)),
      List.of(new Translation2d(5.387, 1.710)),
      new Pose2d(1.454, 1.483, Rotation2d.fromDegrees(225)),
      false);

  public static final Trajectory kHPCargoToRightScoringLocation = createTrajectory(
      new Pose2d(1.454, 1.483, Rotation2d.fromDegrees(225)),
      new Pose2d(5.060, 1.852, Rotation2d.fromDegrees(222)),
      true);

  public static final Trajectory kLTarmacMLCornerToTopCargo = createTrajectory(
      new Pose2d(6.186, 5.233, Rotation2d.fromDegrees(135)),
      new Pose2d(5.306, 5.900, Rotation2d.fromDegrees(135)),
      false);

  public static final Trajectory kTopCargoToHPCargo = createTrajectory(
      new Pose2d(5.306, 5.900, Rotation2d.fromDegrees(135)),
      List.of(new Translation2d(4.204, 5.081)),
      new Pose2d(1.454, 1.483, Rotation2d.fromDegrees(225)),
      false);

  public static final Trajectory kHPCargoToLeftScoringLocation = createTrajectory(
      new Pose2d(1.454, 1.483, Rotation2d.fromDegrees(225)),
      new Pose2d(4.653, 4.579, Rotation2d.fromDegrees(205)),
      true);

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

  public static class CircularRegion {
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
