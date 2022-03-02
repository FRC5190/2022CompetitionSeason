package org.ghrobotics.frc2022.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AnalogInput;

public class Feeder extends SubsystemBase{
    //Motor Controllers
    private final CANSparkMax feederMotor_;
    private final CANSparkMax bridgeMotor_;

    //Sensors
    private final AnalogInput intakeSensor_;
    private final AnalogInput exitSensor_;

    //IO
    private final PeriodicIO io_ = new PeriodicIO();

    /**
   * Constructs an instance of the Feeder subsystem. Only one instance of this subsystem should
   * be created in the main Robot class and references to this instance should be passed around
   * the robot code.
   */
   public Feeder(){
       //Initialize motor controller
       feederMotor_ = new CANSparkMax(Constants.kFeederId, MotorType.kBrushless);
       feederMotor_.restoreFactoryDefaults();
       feederMotor_.setIdleMode(IdleMode.kCoast);
       feederMotor_.enableVoltageCompensation(12);
       feederMotor_.setInverted(false);

       bridgeMotor_ = new CANSparkMax(Constants.kBridgeId, MotorType.kBrushless);
       bridgeMotor_.restoreFactoryDefaults();
       bridgeMotor_.setIdleMode(IdleMode.kCoast);
       bridgeMotor_.enableVoltageCompensation(12);
       bridgeMotor_.setInverted(false);

       //Initialize sensors
       intakeSensor_ = new AnalogInput(kIntakeSensorId);
       exitSensor_ = new AnalogInput(kExitSensorId);  
   }

    public void periodic(){
        feederMotor_.set(io_.demand);
        bridgeMotor_.set(io_.demand);
    }

    /**
    * Sets the % output on the intake.
    *
    * @param value The % output in [-1, 1].
    */
    public void setPercent(double value) {
        io_.demand = value;
    }

    public static class PeriodicIO{
        //Outputs
        double demand;
    }

    public static class Constants{
        //Motor Controllers
        public static final int kFeederId = 0;
        public static final int kBridgeId = 0;

        //Sensors
        public static final int kIntakeSensorId = 0;
        public static final int kExitSensorId = 0;
    }
}