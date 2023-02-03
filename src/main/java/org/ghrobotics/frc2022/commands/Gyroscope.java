package org.ghrobotics.frc2022.commands;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.ctre.phoenix.sensors.BasePigeonSimCollection;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.ghrobotics.frc2022.subsystems.Drivetrain;
import org.ghrobotics.frc2022.RobotState;


public class Gyroscope extends CommandBase {
    private double motorOutput3;
    private double [] ypr = new double[3];
    private final RobotState robotstate = new RobotState();
    Drivetrain drivetrain = new Drivetrain(robotstate);
    WPI_PigeonIMU pigeon = new WPI_PigeonIMU(0);
    public boolean onRamp = false;

    public Gyroscope () {

    }

    @Override
    public void execute(){

        pigeon.getYawPitchRoll(ypr);

        double rollValue = ypr[2] + 0.175;

        System.out.println("Pigeon Roll is: " + ypr[2]);

        if (rollValue > 1 || rollValue < 1){
            onRamp = true;
        }

        if (onRamp){
            if (rollValue > 0.75 || rollValue < -0.75){

                if (rollValue <0){
                    rollValue = rollValue * -1;
                    motorOutput3 = (Math.pow(1.01, rollValue)-1)*100;
                    System.out.println("Motor Output should be: " + (-1 * motorOutput3) + "%");
                } else {
                    motorOutput3 = (Math.pow(1.01, rollValue)-1)*100;
                    System.out.println("Motor Output shoud be: " + motorOutput3 + "%");
                }
                drivetrain.setPercent (motorOutput3, motorOutput3);
            } else {
                drivetrain.setPercent (0, 0);
            }
        }
    addRequirements (drivetrain);
}
    public boolean isStarted (){
        return true;
    }
}