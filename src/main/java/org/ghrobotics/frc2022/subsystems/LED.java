package org.ghrobotics.frc2022.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
  // LED and LED buffer.
  private final AddressableLED led_;
  private final AddressableLEDBuffer led_buffer_;

  // Keeps track of rainbow pattern.
  int rainbow_first_pixel_hue_ = 0;

  // IO
  private OutputType output_type_ = OutputType.RAINBOW;
  private StandardLEDOutput standard_led_output_;

  /**
   * Constructs an instance of the LED subsystem. Only one instance of this subsystem should
   * be created in the main Robot class and references to this instance should be passed around
   * the robot code.
   */
  public LED() {
    // Initialize LED and LED buffer.
    led_buffer_ = new AddressableLEDBuffer(Constants.kBufferSize);
    led_ = new AddressableLED(Constants.kPortId);
    led_.setLength(led_buffer_.getLength());
    led_.start();
  }

  /**
   * Runs periodically every 20 ms. Here, the LED output is set.
   */
  @Override
  public void periodic() {
    // Write outputs.
    switch (output_type_) {
      case STANDARD:
        // Calculate the % of time that we remain on.
        double total_cycle_time = standard_led_output_.on_time + standard_led_output_.off_time;
        double on_ratio = standard_led_output_.on_time / total_cycle_time;

        // Calculate which % of the time we are in.
        double remainder = Timer.getFPGATimestamp() % total_cycle_time;
        boolean on = remainder / total_cycle_time <= on_ratio;

        // Set the buffer.
        for (int i = 0; i < Constants.kBufferSize; i++) {
          led_buffer_.setLED(i, on ? standard_led_output_.c : Color.kBlack);
        }
        break;

      case RAINBOW:
        // Calculate and fill buffer with rainbow.
        setRainbow();
        break;
    }

    // Set the LED data.
    led_.setData(led_buffer_);
  }


  /**
   * Sets the standard (solid color for duration of time) LED output.
   *
   * @param output The LED output.
   */
  public void setOutput(StandardLEDOutput output) {
    output_type_ = OutputType.STANDARD;
    standard_led_output_ = output;
  }

  /**
   * Sets the special (non-standard) LED output.
   *
   * @param type The type of special LED output.
   */
  public void setOutput(OutputType type) {
    output_type_ = type;
    standard_led_output_ = StandardLEDOutput.BLANK;
  }

  /**
   * Calculates a moving rainbow and sets the LED buffer.
   */
  private void setRainbow() {
    for (int i = 0; i < Constants.kBufferSize; i++) {
      // Calculate the hue: hue is easier for rainbows because the color shape is a circle, so
      // only one value needs to precess.
      int hue = (rainbow_first_pixel_hue_ + (i * 180 / Constants.kBufferSize)) % 180;

      // Set the value.
      led_buffer_.setHSV(i, hue, 255, 128);
    }

    // Increase by 3 to make the rainbow move.
    rainbow_first_pixel_hue_ += 3;

    // Check bounds.
    rainbow_first_pixel_hue_ %= 180;
  }

  public enum OutputType {
    STANDARD, RAINBOW
  }

  public enum StandardLEDOutput {
    // Blank:
    BLANK(Color.kBlack, 1.0, 0.0),

    // Climb Indicators:
    CLIMBING(Color.kOrange, 0.75, 0.75),
    CLIMB_RESETTING(Color.kOrange, 0.1, 0.1),

    // Turret Zero Indicators:
    NO_TURRET_ZERO(Color.kRed, 0.5, 0.5),
    TURRET_ZEROING(Color.kHotPink, 0.075, 0.075),

    // Limelight Indicators:
    NO_LIMELIGHT(Color.kPurple, 0.5, 0.5),

    // Superstructure Indicators:
    AUTOMATIC_SCORING(Color.kGreen, 0.075, 0.075),
    MANUAL_SCORING(Color.kBlue, 0.075, 0.075);


    // Stores the color and on percentage for the current output.
    Color c;
    double on_time;
    double off_time;

    /**
     * Constructs an enum instance of a standard LED output (solid color with an on-off duration).
     *
     * @param c        The color of the LED.
     * @param on_time  The length of time that the LED is on.
     * @param off_time The length of time that the LED is off.
     */
    StandardLEDOutput(Color c, double on_time, double off_time) {
      this.c = c;
      this.on_time = on_time;
      this.off_time = off_time;
    }
  }

  public static class Constants {
    public static final int kPortId = 0;
    public static final int kBufferSize = 24;
  }
}