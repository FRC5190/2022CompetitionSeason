package org.ghrobotics.lib.interpolation;

import java.util.Map;
import java.util.Optional;
import java.util.TreeMap;

/**
 * Used to create lookup tables from double -> double. This can be used for distance -> rpm or
 * distance -> hood angle calculations in the robot code.
 */
public class LookupTable {
  private final TreeMap<Double, Double> tree_map_;
  private final double default_value_;

  public LookupTable(double default_value) {
    tree_map_ = new TreeMap<>();
    default_value_ = default_value;
  }

  public void put(double key, double value) {
    tree_map_.put(key, value);
  }

  public double get(double key) {
    // If the map is empty, return default value.
    if (tree_map_.isEmpty())
      return default_value_;

    // If the exact value exists in the map, just return that.
    Optional<Double> value = Optional.ofNullable(tree_map_.get(key));
    if (value.isPresent())
      return value.get();

    // Find the floor and ceiling entries for the given key.
    Map.Entry<Double, Double> ceiling_entry = tree_map_.ceilingEntry(key);
    Map.Entry<Double, Double> floor_entry = tree_map_.floorEntry(key);

    // If there is no upper bound, return highest element.
    if (ceiling_entry == null)
      return floor_entry.getValue();

    // If there is no lower bound, return lowest element.
    if (floor_entry == null)
      return ceiling_entry.getValue();

    // If we have both upper and lower bounds, interpolate between the two and return.
    return floor_entry.getValue() + (ceiling_entry.getValue() - floor_entry.getValue()) *
        ((key - floor_entry.getKey()) / (ceiling_entry.getKey() - floor_entry.getKey()));
  }
}
