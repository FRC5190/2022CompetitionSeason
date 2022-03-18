package org.ghrobotics.frc2022.commands;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.BooleanSupplier;
import org.ghrobotics.frc2022.subsystems.Turret;

public class TurretZero extends CommandBase {
  // Reference to subsystem, override button, and timer.
  private final Turret turret_;
  private final BooleanSupplier override_button_;
  private final Timer timer_;

  public TurretZero(Turret turret, BooleanSupplier override_button) {
    // Assign member variables.
    turret_ = turret;
    override_button_ = override_button;

    // Initialize timer.
    timer_ = new Timer();

    // Add subsystem requirements.
    addRequirements(turret_);
  }

  @Override
  public void initialize() {
    timer_.start();
  }

  @Override
  public void execute() {
    // If we don't have the hall sensor or the velocity isn't zero, then we are not zeroing.
    if ((!turret_.getHallSensor() && !override_button_.getAsBoolean()) ||
        Math.abs(turret_.getVelocity()) > 0.2) {
      // Reset timer and say that we are not zeroing.
      timer_.reset();
      turret_.setStatus(Turret.Status.NO_ZERO);
    } else {
      // We are zeroing.
      turret_.setStatus(Turret.Status.ZEROING);
    }
  }

  @Override
  public void end(boolean interrupted) {
    // Stop and reset the timer.
    timer_.stop();
    timer_.reset();

    // If we were not interrupted, zero the turret.
    if (!interrupted)
      turret_.zero();
  }

  @Override
  public boolean isFinished() {
    // We are done when we have been zeroing for 3 seconds (or if we are in simulation).
    return timer_.hasElapsed(3) || RobotBase.isSimulation();
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
