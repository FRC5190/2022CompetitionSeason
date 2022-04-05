package org.ghrobotics.lib.telemetry;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class MissionControl {

  private static final Map<String, DoubleSupplier> double_entries_ = new HashMap<>();
  private static final Map<String, BooleanSupplier> bool_entries_ = new HashMap<>();
  private static final Map<String, Supplier<String>> string_entries_ = new HashMap<>();

  private static final NetworkTable robot_table_ =
      NetworkTableInstance.getDefault().getTable("robot");

  public static void addDouble(String k, DoubleSupplier v) {
    double_entries_.put(k, v);
  }

  public static void addBoolean(String k, BooleanSupplier v) {
    bool_entries_.put(k, v);
  }

  public static void addString(String k, Supplier<String> v) {
    string_entries_.put(k, v);
  }

  public static void update() {
    double_entries_.forEach((k, v) -> robot_table_.getEntry(k).setDouble(v.getAsDouble()));
    bool_entries_.forEach((k, v) -> robot_table_.getEntry(k).setBoolean(v.getAsBoolean()));
    string_entries_.forEach((k, v) -> robot_table_.getEntry(k).setString(v.get()));
  }
}
