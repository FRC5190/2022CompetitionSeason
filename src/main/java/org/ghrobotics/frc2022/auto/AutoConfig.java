package org.ghrobotics.frc2022.auto;

import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;

public class AutoConfig {
  // Constraints
  public static final double kMaxVelocity = 3.0;
  public static final double kMaxAcceleration = 1.5;
  public static final double kMaxCentripetalAcceleration = 1.5;

  public static final TrajectoryConfig kForwardConfig =
      new TrajectoryConfig(kMaxVelocity, kMaxAcceleration)
          .addConstraint(new CentripetalAccelerationConstraint(kMaxCentripetalAcceleration));

  public static final TrajectoryConfig kReverseConfig =
      new TrajectoryConfig(kMaxVelocity, kMaxAcceleration)
          .setReversed(true)
          .addConstraint(new CentripetalAccelerationConstraint(kMaxCentripetalAcceleration));
}
