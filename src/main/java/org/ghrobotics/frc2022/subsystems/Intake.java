package org.ghrobotics.frc2022.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static com.revrobotics.CANSparkMax.IdleMode;
import static com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Intake extends SubsystemBase {
  // Motor Controllers
  private final CANSparkMax intake_leader_;
  private final CANSparkMax bridge_leader_;
  private final CANSparkMax bridge_follower_;

  // Pneumatics
  private final DoubleSolenoid pivot_;

  // IO
  private final PeriodicIO io_ = new PeriodicIO();

  /**
   * Constructs an instance of the Intake subsystem. Only one instance of this subsystem should
   * be created in the main Robot class and references to this instance should be passed around
   * the robot code.
   */
  public Intake() {
    // Initialize motor controllers.
    intake_leader_ = new CANSparkMax(Constants.kIntakeLeaderId, MotorType.kBrushless);
    intake_leader_.restoreFactoryDefaults();
    intake_leader_.setIdleMode(IdleMode.kCoast);
    intake_leader_.enableVoltageCompensation(12);
    intake_leader_.setSmartCurrentLimit(Constants.kIntakeCurrentLimit);
    intake_leader_.setInverted(true);

    bridge_leader_ = new CANSparkMax(Constants.kBridgeLeaderId, MotorType.kBrushless);
    bridge_leader_.restoreFactoryDefaults();
    bridge_leader_.setIdleMode(IdleMode.kCoast);
    bridge_leader_.enableVoltageCompensation(12);
    bridge_leader_.setSmartCurrentLimit(Constants.kBridgeCurrentLimit);
    bridge_leader_.setInverted(false);

    bridge_follower_ = new CANSparkMax(Constants.kBridgeFollowerId, MotorType.kBrushless);
    bridge_follower_.restoreFactoryDefaults();
    bridge_follower_.setIdleMode(IdleMode.kCoast);
    bridge_follower_.enableVoltageCompensation(12);
    bridge_follower_.setSmartCurrentLimit(Constants.kBridgeCurrentLimit);
    bridge_follower_.follow(bridge_leader_, true);

    // Initialize pneumatics.
    pivot_ = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.kPivotForwardId,
        Constants.kPivotReverseId);
  }

  /**
   * This method runs periodically every 20 ms. Here, all sensor values are read and all motor
   * outputs should be set.
   */
  @Override
  public void periodic() {
    // Write outputs.
    if (io_.wants_pneumatics_update) {
      io_.wants_pneumatics_update = false;
      pivot_.set(io_.pivot_value ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
    }

    intake_leader_.set(io_.demand);
    bridge_leader_.set(io_.demand);
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
    boolean pivot_value = true;
    boolean wants_pneumatics_update;
  }

  public static class Constants {
    // Motor Controllers
    public static final int kIntakeLeaderId = 9;
    public static final int kBridgeLeaderId = 10;
    public static final int kBridgeFollowerId = 11;

    // Pneumatics
    public static final int kPivotForwardId = 4;
    public static final int kPivotReverseId = 5;

    // Current Limits
    public static final int kIntakeCurrentLimit = 80;
    public static final int kBridgeCurrentLimit = 20;
  }
}