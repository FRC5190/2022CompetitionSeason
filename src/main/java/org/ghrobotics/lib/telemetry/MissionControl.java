package org.ghrobotics.lib.telemetry;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * This class is used for interaction with the FRC 5190 dashboard -- Mission Control. Telemetry
 * can be sent and received. via the helper methods contained in this class.
 */
public class MissionControl {
  // Outbound Entries
  private static final Map<String, DoubleSupplier> double_entries_ = new HashMap<>();
  private static final Map<String, BooleanSupplier> bool_entries_ = new HashMap<>();
  private static final Map<String, Supplier<String>> string_entries_ = new HashMap<>();

  // Main NetworkTable
  private static final NetworkTable robot_table_ =
      NetworkTableInstance.getDefault().getTable("robot");

  /**
   * Adds a numerical value to the dashboard.
   *
   * @param k The key to use.
   * @param v Supplier for the numerical value.
   */
  public static void addDouble(String k, DoubleSupplier v) {
    double_entries_.put(k, v);
  }

  /**
   * Adds a boolean value to the dashboard.
   *
   * @param k The key to use.
   * @param v Supplier for the boolean value.
   */
  public static void addBoolean(String k, BooleanSupplier v) {
    bool_entries_.put(k, v);
  }

  /**
   * Adds a string value to the dashboard.
   *
   * @param k The key to use.
   * @param v Supplier for the string value.
   */
  public static void addString(String k, Supplier<String> v) {
    string_entries_.put(k, v);
  }

  /**
   * Retrieves a numerical value from the dashboard.
   * @param k The key to use.
   * @return The numerical value from the dashboard.
   */
  public static double getDouble(String k) {
    return robot_table_.getEntry(k).getDouble(0);
  }

  /**
   * Retrieves a boolean value from the dashboard.
   * @param k The key to use.
   * @return The boolean value from the dashboard.
   */
  public static boolean getBoolean(String k) {
    return robot_table_.getEntry(k).getBoolean(false);
  }

  /**
   * Retrieves a string value from the dashboard.
   * @param k The key to use.
   * @return The string value from the dashboard.
   */
  public static String getString(String k) {
    return robot_table_.getEntry(k).getString("");
  }

  /**
   * Updates and sends the latest requested entries to the dashboard. This must be called from
   * robotPeriodic() or at some periodic rate.
   */
  public static void update() {
    // Iterate through outbound entries and set values.
    double_entries_.forEach((k, v) -> robot_table_.getEntry(k).setDouble(v.getAsDouble()));
    bool_entries_.forEach((k, v) -> robot_table_.getEntry(k).setBoolean(v.getAsBoolean()));
    string_entries_.forEach((k, v) -> robot_table_.getEntry(k).setString(v.get()));
  }
}
