package org.ghrobotics.frc2022.planners;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.IOException;
import java.util.Scanner;
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
        Units.radiansPerSecondToRotationsPerMinute(default_shooter_rpm));
    hood_angle_table_ = new LookupTable(default_hood_angle);

    // Read tables from filesystem (need try-catch due to checked exception).
    try {
      // Create and configure scanner to read file.
      Scanner scanner = new Scanner(
          Filesystem.getDeployDirectory().toPath().resolve("shooter_lut.csv"));

      // Read each line of the CSV and add to lookup table.
      while (scanner.hasNextLine()) {
        // Split each line into distance, rpm, and exit angle values.
        String[] values = scanner.nextLine().split(",");

        // Parse numerical values.
        double distance = Double.parseDouble(values[0]);
        double rpm = Double.parseDouble(values[1]);
        double exit_angle = Double.parseDouble(values[2]);

        // Populate tables.
        shooter_speed_table.put(distance, Units.rotationsPerMinuteToRadiansPerSecond(rpm * 1.065));
        hood_angle_table_.put(distance, Math.toRadians(90 - exit_angle + 2.1));
      }

    } catch (IOException ex) {
      // Report IO error to Driver Station.
      DriverStation.reportError("Could not read shooter lookup table!", false);
      ex.printStackTrace();
    }
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

