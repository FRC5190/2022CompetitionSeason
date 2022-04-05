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
    bridge_follower_.follow(bridge_leader_, true);

    // Initialize pneumatics.
    pivot_ = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.kPivotForwardId,
        Constants.kPivotReverseId);
  }

  /**
   * This method runs periodically every 20 ms. Here, all sensor values are read and all motor
   * outputs should be set.
   */
  @Override
  public void periodic() {
    // Read inputs.
    io_.intake_supply_current = intake_leader_.getOutputCurrent();
    io_.bridge_leader_supply_current = bridge_leader_.getOutputCurrent();
    io_.bridge_follower_supply_current = bridge_follower_.getOutputCurrent();

    // Write outputs.
    if (io_.wants_pneumatics_update) {
      io_.wants_pneumatics_update = false;
      pivot_.set(io_.pivot_value ? DoubleSolenoid.Value.kReverse : DoubleSolenoid.Value.kForward);
    }

    intake_leader_.set(io_.intake_bridge_demand);
    bridge_leader_.set(io_.intake_bridge_demand);
  }

  /**
   * Sets the % output on the intake.
   *
   * @param value The % output in [-1, 1].
   */
  public void setPercent(double value) {
    io_.intake_bridge_demand = value;
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
    // Inputs
    double intake_supply_current;
    double bridge_leader_supply_current;
    double bridge_follower_supply_current;

    // Outputs
    double intake_bridge_demand;

    boolean pivot_value = false;
    boolean wants_pneumatics_update = true;
  }

  public static class Constants {
    // Motor Controllers
    public static final int kIntakeLeaderId = 9;
    public static final int kBridgeLeaderId = 10;
    public static final int kBridgeFollowerId = 11;

    // Pneumatics
    public static final int kPivotForwardId = 5;
    public static final int kPivotReverseId = 6;

    // Current Limits
    public static final int kIntakeCurrentLimit = 80;
    public static final int kBridgeCurrentLimit = 20;
  }
}