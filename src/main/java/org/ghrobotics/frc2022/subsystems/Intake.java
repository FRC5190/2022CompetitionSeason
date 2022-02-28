package org.ghrobotics.frc2022.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.Solenoid;

public class Intake extends SubsystemBase{
    //Motor controllers
    private final CANSparkMax leader;

    //Pneumatics
    private final Solenoid pivot;

    //io
    private final PeriodicIO io_ = new PeriodicIO();

    //Initialize
    leader = new CANSparkMax(Constants.kIntakeId, MotorType.kBrushless);
    pivot = new Solenoid(PneumaticsModuleType.REVPH, Constants.kPivotId);

    public void periodic() {
        leader.set(io_.demand);
    }

    public void setPercent(double value){
       io_.demand = value;
    }

    public static class PeriodicIO{
        double demand;
    }

    //Constants
    public static final int kPivotId = 0;
    public static final int kIntakeId = 0;

}