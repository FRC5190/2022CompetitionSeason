package org.ghrobotics.frc2022.commands;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.BasePigeonSimCollection;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.ghrobotics.frc2022.subsystems.Drivetrain;
import org.ghrobotics.frc2022.RobotState;


public class Gyroscope extends CommandBase {
    // private double motorOutput3;
    //private double [] ypr = new double[3];
    private final RobotState robotstate = new RobotState();
    Drivetrain drivetrain = new Drivetrain(robotstate);
    // WPI_PigeonIMU pigeon = new WPI_PigeonIMU(0);
    public boolean onRamp = false;

    // Assigns variables and calls the _pigeon object
    PigeonIMU _pigeon = new PigeonIMU(17);
    int _loopCount = 0;
    double rollValue;
    double motorOutput;
    boolean finished = false;
    // If it is within 8 degree it should stop
    double slackAngle = 8;
  
    boolean onPlatform = false;
    public Gyroscope () {
        
    }

    @Override
    public void execute(){

        System.out.println("The angle is: " + rollValue);
        // Checks if the rollValue is negative and if it is, it will multiply it by -1 to make it positive
        // I used the 1.01^x formula to make the motor output increase exponentially and I checked on Desmos and it should work
        // I also added a 100 to the end to make the motor output a percentage
        // I don't know how the negative motor output will work, but we will see

        if((rollValue < -slackAngle) && !finished)
        {
            rollValue = rollValue * -1;
            // motorOutput = (Math.pow(1.007, rollValue) - 1);
            rollValue = Math.toRadians(rollValue);
            motorOutput = Math.sin(rollValue)/3;
            System.out.println("Motor Output should be: " + (1 * motorOutput) + "%");
        }
        else if ((rollValue > slackAngle) && !finished)
        {
            // motorOutput = (Math.pow(1.007, rollValue) - 1);
            rollValue = Math.toRadians(rollValue);
            motorOutput = Math.sin(rollValue)/3;
            System.out.println("Motor Output should be: " + (1 * motorOutput) + "%");
        } else{
            finished = true;
            drivetrain.setVelocity(0, 0);
            System.out.println("Speed is zero");
        }
        drivetrain.setVelocity(-motorOutput, -motorOutput);

    }
    
}