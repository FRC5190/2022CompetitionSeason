package org.ghrobotics.frc2022.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.math.trajectory.constraint.MaxVelocityConstraint;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import java.util.List;

public class AutoConfig {
  // Constraints
  public static final double kMaxVelocity = 3.0;
  public static final double kMaxAcceleration = 1.8;
  public static final double kMaxCentripetalAcceleration = 1.5;

  public static final TrajectoryConfig kForwardConfig =
      new TrajectoryConfig(kMaxVelocity, kMaxAcceleration)
          .addConstraint(new CentripetalAccelerationConstraint(kMaxCentripetalAcceleration));

  public static final TrajectoryConfig kReverseConfig =
      new TrajectoryConfig(kMaxVelocity, kMaxAcceleration)
          .setReversed(true)
          .addConstraint(new CentripetalAccelerationConstraint(kMaxCentripetalAcceleration));

  // Trajectories
  public static final Trajectory kRightStartToBottomCargo = TrajectoryGenerator.generateTrajectory(
      new Pose2d(7.624, 1.880, Rotation2d.fromDegrees(271.5)), List.of(),
      new Pose2d(7.602, 1.030, Rotation2d.fromDegrees(270)),
      kForwardConfig
  );

  public static final Trajectory kBottomCargoToIntermediateA =
      TrajectoryGenerator.generateTrajectory(
          new Pose2d(7.602, 1.030, Rotation2d.fromDegrees(270)), List.of(),
          new Pose2d(8.298, 1.256, Rotation2d.fromDegrees(182)),
          kReverseConfig
      );

  public static final Trajectory kIntermediateAToMiddleCargo =
      TrajectoryGenerator.generateTrajectory(
          new Pose2d(8.298, 1.256, Rotation2d.fromDegrees(182)), List.of(),
          new Pose2d(5.196, 1.992, Rotation2d.fromDegrees(132)),
          kForwardConfig
      );

  public static final Trajectory kMiddleCargoToHP =
      TrajectoryGenerator.generateTrajectory(
          new Pose2d(5.196, 1.992, Rotation2d.fromDegrees(132)), List.of(),
          new Pose2d(1.453, 1.483, Rotation2d.fromDegrees(225)),
          kForwardConfig
      );

  public static final Trajectory kHPToIntermediateB = TrajectoryGenerator.generateTrajectory(
      new Pose2d(1.453, 1.483, Rotation2d.fromDegrees(225)), List.of(),
      new Pose2d(5.086, 2.330, Rotation2d.fromDegrees(180)),
      kReverseConfig
  );
}
