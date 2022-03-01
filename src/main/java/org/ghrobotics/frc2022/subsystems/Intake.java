package org.ghrobotics.frc2022.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static com.revrobotics.CANSparkMax.IdleMode;
import static com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Intake extends SubsystemBase {
  // Motor Controllers
  private final CANSparkMax leader_;

  // Pneumatics
  private final Solenoid pivot_;

  // IO
  private final PeriodicIO io_ = new PeriodicIO();

  /**
   * Constructs an instance of the Intake subsystem. Only one instance of this subsystem should
   * be created in the main Robot class and references to this instance should be passed around
   * the robot code.
   */
  public Intake() {
    // Initialize motor controller.
    leader_ = new CANSparkMax(Constants.kLeaderId, MotorType.kBrushless);
    leader_.restoreFactoryDefaults();
    leader_.setIdleMode(IdleMode.kBrake);
    leader_.enableVoltageCompensation(12);
    leader_.setInverted(false);

    // Initialize pneumatics.
    pivot_ = new Solenoid(PneumaticsModuleType.REVPH, Constants.kPivotId);
  }

  /**
   * This method runs periodically every 20 ms. Here, all sensor values are read and all motor
   * outputs should be set.
   */
  public void periodic() {
    // Write outputs.
    if (io_.wants_pneumatics_update) {
      io_.wants_pneumatics_update = false;
      pivot_.set(io_.pivot_value);
    }

    leader_.set(io_.demand);
  }

  /**
   * Sets the % output on the intake.
   *
   * @param value The % output in [-1, 1].
   */
  public void setPercent(double value) {
    io_.demand = value;
  }

  /**
   * Sets the intake pivot.
   *
   * @param value The pivot value; true if the intake should be extended.
   */
  public void setPivot(boolean value) {
    io_.wants_pneumatics_update = value != io_.pivot_value;
    io_.pivot_value = value;
  }

  public static class PeriodicIO {
    // Outputs
    double demand;
    boolean pivot_value;
    boolean wants_pneumatics_update;
  }

  public static class Constants {
    // Motor Controllers
    public static final int kLeaderId = 9;

    // Pneumatics
    public static final int kPivotId = 0;
  }
}