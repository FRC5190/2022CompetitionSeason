package org.ghrobotics.frc2022.subsystems;

import com.revrobotics.ColorMatch;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.ghrobotics.lib.sensor.PicoColorSensor;
import static edu.wpi.first.math.filter.Debouncer.DebounceType;

public class CargoTracker extends SubsystemBase {
  // Shooter and Intake Subsystem Reference
  private final Intake intake_;
  private final Shooter shooter_;

  // Color Sensors
  private final PicoColorSensor color_sensors_;

  // Color Matcher
  private final ColorMatch color_match_;

  // Debouncer
  private final Debouncer lower_sensor_debouncer_;
  private final Debouncer upper_sensor_debouncer_;

  // Subsystem State
  private boolean ball_at_lower_ = false;
  private boolean ball_at_upper_ = false;
  private Color lower_color_ = null;
  private Color upper_color_ = null;

  // Cargo Tracker State. Index 0 is closest to shooter.
  private final Color[] cargo_;

  // IO
  private final PeriodicIO io_ = new PeriodicIO();

  /**
   * Constructs an instance of the Cargo Tracker subsystem. This is a somewhat unconventional
   * subsystem and simply exists to track cargo as it enters and leaves the robot.
   *
   * @param intake  Reference to intake subsystem.
   * @param shooter Reference to shooter subsystem.
   */
  public CargoTracker(Intake intake, Shooter shooter) {
    // Assign robot state and shooter references.
    intake_ = intake;
    shooter_ = shooter;

    // Initialize color sensors.
    color_sensors_ = new PicoColorSensor();

    // Initialize and configure color match.
    color_match_ = new ColorMatch();
    color_match_.addColorMatch(Constants.kRed);
    color_match_.addColorMatch(Constants.kBlue);

    // Initialize debouncers.
    lower_sensor_debouncer_ = new Debouncer(Constants.kDebounceTime, DebounceType.kBoth);
    upper_sensor_debouncer_ = new Debouncer(Constants.kDebounceTime, DebounceType.kBoth);

    // Initialize cargo tracker state.
    cargo_ = new Color[]{null, null};
  }

  @Override
  public void periodic() {
    // Read inputs.
    color_sensors_.getRawColor0(io_.lower_sensor);
    color_sensors_.getRawColor1(io_.upper_sensor);
    io_.lower_sensor_prox = color_sensors_.getProximity0();
    io_.upper_sensor_prox = color_sensors_.getProximity1();

    // Store old values.
    boolean prev_ball_at_lower = ball_at_lower_;
    boolean prev_ball_at_upper = ball_at_upper_;

    // Calculate whether there is a ball in front of any sensor.
    ball_at_lower_ = lower_sensor_debouncer_
        .calculate(io_.lower_sensor_prox > Constants.kLowerProximityThreshold);
    ball_at_upper_ = io_.upper_sensor_prox > Constants.kUpperProximityThreshold;

//    SmartDashboard.putNumber("Lower Sensor Prox", io_.lower_sensor_prox);
//    SmartDashboard.putNumber("Upper Sensor Prox", io_.upper_sensor_prox);
//
//    // If there are balls, get the color of the ball.
//    lower_color_ = ball_at_lower_ ?
//        color_match_.matchClosestColor(toColor(io_.lower_sensor)).color : null;
//    upper_color_ = ball_at_upper_ ?
//        color_match_.matchClosestColor(toColor(io_.upper_sensor)).color : null;
//
//    // The only way for cargo to enter is through the lower sensor. If there was previously no
//    // ball but there is a ball now, it entered. We just make sure that this isn't a bad sensor
//    // reading by also making sure that the intake is running.
//    if (!prev_ball_at_lower && ball_at_lower_ && intake_.getIntakePercent() > 0)
//      cargo_[Math.min(1, getCargoCount())] = lower_color_;
//
//    // If there is a ball sitting at the upper sensor AND cargo_[0] exists, update its color value
//    // for reinforcement.
//    if (cargo_[0] != null && ball_at_upper_)
//      cargo_[0] = upper_color_;
//
//    // The only way for cargo to exit is through the upper sensor. If the ball was previously at
//    // the upper sensor but is no longer there, the ball must have exited through the shooter. We
//    // just make sure that this isn't a bad sensor reading by also making sure that the shooter
//    // has a positive velocity.
//    if (prev_ball_at_upper && !ball_at_upper_ && shooter_.getVelocity() > 0) {
//      // Shift everything to the left.
//      cargo_[0] = cargo_[1];
//      cargo_[1] = null;
//    }
  }

  /**
   * Resets the state of the cargo tracking.
   *
   * @param cargo The new cargo array to use, with 0-th index being closest to the shooter.
   */
  public void reset(Color[] cargo) {
    if (cargo.length != 2)
      throw new IllegalArgumentException("The cargo array has the wrong length.");

    cargo_[0] = cargo[0];
    cargo_[1] = cargo[1];
  }

  /**
   * Returns whether there is a ball at the lower sensor.
   *
   * @return Whether there is a ball at the lower sensor.
   */
  public boolean getBallAtLower() {
    return ball_at_lower_;
  }

  /**
   * Returns whether there is a ball at the upper sensor.
   *
   * @return Whether there is a ball at the upper sensor.
   */
  public boolean getBallAtUpper() {
    return ball_at_upper_;
  }

  /**
   * Returns the color of the next ball that is going to enter the shooter.
   *
   * @return The color of the next ball that is going to enter the shooter.
   */
  public Color getNextBall() {
    return cargo_[0];
  }

  /**
   * Returns the number of cargo inside the robot at this time.
   *
   * @return The number of cargo inside the robot at this time.
   */
  public int getCargoCount() {
    int count = 0;
    for (int i = 0; i < 2; i++)
      if (cargo_[i] != null)
        count++;

    return count;
  }

  /**
   * Converts the PicoColorSensor.RawColor struct to a WPILib Color struct.
   *
   * @param raw_color Instance of the raw color struct.
   * @return The converted WPILib converted color object.
   */
  private Color toColor(PicoColorSensor.RawColor raw_color) {
    double mag = raw_color.red + raw_color.blue + raw_color.green;
    return new Color(raw_color.red / mag, raw_color.green / mag, raw_color.blue / mag);
  }

  public static class PeriodicIO {
    // Inputs
    PicoColorSensor.RawColor lower_sensor = new PicoColorSensor.RawColor();
    PicoColorSensor.RawColor upper_sensor = new PicoColorSensor.RawColor();
    int lower_sensor_prox;
    int upper_sensor_prox;
  }

  public static class Constants {
    // Color Matches
    public static final Color kRed = new Color(1, 0, 0);
    public static final Color kBlue = new Color(0, 0, 1);

    // Debouncer
    public static final double kDebounceTime = 0.09;

    // Thresholds
    public static final int kUpperProximityThreshold = 125;
    public static final int kLowerProximityThreshold = 275;
  }
}
