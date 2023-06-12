package frc.robot.sensors;

import com.ctre.phoenix.sensors.PigeonIMU;

public class Pigeon {

    private PigeonIMU pigeon = new PigeonIMU(8);

    public double getHeading(){
        return -pigeon.getYaw();
    }

    public void reset(){
        pigeon.setYaw(0);
    }
    
}
