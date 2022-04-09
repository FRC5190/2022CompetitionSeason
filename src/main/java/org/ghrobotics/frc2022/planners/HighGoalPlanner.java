package org.ghrobotics.frc2022.planners;

import edu.wpi.first.math.util.Units;
import org.ghrobotics.lib.interpolation.LookupTable;

/**
 * Handles planning of shots into the high goal. This is achieved via a lookup table to
 * pre-generated values from a ballistics calculator or pre-tuned values from the playing field.
 */
public class HighGoalPlanner {
  // Lookup Tables
  private final LookupTable shooter_speed_table;
  private final LookupTable hood_angle_table_;

  /**
   * Constructs an instance of the high goal planner and initializes lookup tables (either
   * programmatically or reading from a file).
   *
   * @param default_shooter_rpm The default shooter rpm to use in case of an error.
   * @param default_hood_angle  The default hood angle to use in case of an error.
   */
  public HighGoalPlanner(double default_shooter_rpm, double default_hood_angle) {
    // Initialize tables.
    shooter_speed_table = new LookupTable(
        Units.rotationsPerMinuteToRadiansPerSecond(default_shooter_rpm));
    hood_angle_table_ = new LookupTable(default_hood_angle);

    // Add values.
    add(3.05, 2000, 17);
    add(4.60, 2000, 19);
    add(6.10, 2425, 26);
    add(8.50, 2525, 30);
  }

  /**
   * Returns the shooter speed (in radians per second) for the given distance.
   *
   * @param distance The distance to the goal in meters.
   * @return The shooter speed.
   */
  public double getShooterSpeed(double distance) {
    return shooter_speed_table.get(distance);
  }

  /**
   * Returns the hood angle (in radians) for the given distance.
   *
   * @param distance The distance to the goal in meters.
   * @return The hood angle.
   */
  public double getHoodAngle(double distance) {
    return hood_angle_table_.get(distance);
  }

  /**
   * Adds an entry to both lookup tables.
   *
   * @param distance  The distance to the goal.
   * @param rpm       The shooter rpm.
   * @param angle_deg The hood angle in degrees.
   */
  private void add(double distance, double rpm, double angle_deg) {
    shooter_speed_table.put(distance, Units.rotationsPerMinuteToRadiansPerSecond(rpm));
    hood_angle_table_.put(distance, Math.toRadians(angle_deg));
  }
}

