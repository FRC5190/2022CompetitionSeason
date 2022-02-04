package org.ghrobotics.frc2022.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless;
import static org.ghrobotics.frc2022.subsystems.Drivetrain.Constants.kLeftFollower;
import static org.ghrobotics.frc2022.subsystems.Drivetrain.Constants.kLeftLeader;
import static org.ghrobotics.frc2022.subsystems.Drivetrain.Constants.kRightFollower;
import static org.ghrobotics.frc2022.subsystems.Drivetrain.Constants.kRightLeader;

public class Drivetrain extends SubsystemBase {
  // Motor Controllers
  private final CANSparkMax left_leader_ = new CANSparkMax(kLeftLeader, kBrushless);
  private final CANSparkMax left_follower_ = new CANSparkMax(kLeftFollower, kBrushless);
  private final CANSparkMax right_leader_ = new CANSparkMax(kRightLeader, kBrushless);
  private final CANSparkMax right_follower_ = new CANSparkMax(kRightFollower, kBrushless);

  // IO
  private final PeriodicIO io_ = new PeriodicIO();

  // Constructor
  public Drivetrain() {
    // Restore factory defaults.
    left_leader_.restoreFactoryDefaults();
    left_follower_.restoreFactoryDefaults();
    right_leader_.restoreFactoryDefaults();
    right_follower_.restoreFactoryDefaults();

    // Invert right side of drivetrain.
    right_leader_.setInverted(true);

    // Set leader-follower relationship.
    left_follower_.follow(left_leader_);
    right_follower_.follow(right_leader_);
  }

  // Runs periodically every 20 ms.
  @Override
  public void periodic() {
    // Read inputs.
    io_.robot_controller_voltage = RobotController.getBatteryVoltage();

    // Write outputs.
    left_leader_.setVoltage(io_.l_voltage);
    right_leader_.setVoltage(io_.r_voltage);
  }

  // Sets the desired motor voltages.
  public void setVoltages(double left, double right) {
    io_.l_voltage = left;
    io_.r_voltage = right;
  }

  // Sets the desired motor percent outputs. This is automatically multiplied by the robot
  // controller voltage to determine the appropriate voltages.
  public void setPercents(double left, double right) {
    setVoltages(left * io_.robot_controller_voltage, right * io_.robot_controller_voltage);
  }


  // IO
  static class PeriodicIO {
    // Inputs
    double robot_controller_voltage;

    // Outputs
    double l_voltage;
    double r_voltage;
  }

  // Constants
  static class Constants {
    // Motor Controller IDs
    public static final int kLeftLeader = 1;
    public static final int kLeftFollower = 2;
    public static final int kRightLeader = 3;
    public static final int kRightFollower = 4;
  }
}
