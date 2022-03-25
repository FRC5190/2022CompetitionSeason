package org.ghrobotics.frc2022;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * Contains constants, measurements, and regions for the FIRST Robotics Competition 2022 game -
 * RAPID REACT.
 */
public class Arena {
  // REGIONS
  // Hangar Zone
  public static Translation2d kHangarTLCorner = new Translation2d(0.00, 8.23);
  public static Translation2d kHangarBRCorner = new Translation2d(3.27, 5.28);

  // Safe Hangar Zone
  public static Translation2d kSafeHangarTLCorner = kHangarTLCorner.plus(new Translation2d(-1, 1));
  public static Translation2d kSafeHangarBRCorner = kHangarBRCorner.plus(new Translation2d(1, -1));

  // POINTS
  // Hangar
  public static Translation2d kHangar = new Translation2d(1.64, 6.75);

  // Goal
  public static Translation2d kGoal = new Translation2d(8.23, 4.11);
}
