package org.firstinspires.ftc.teamcode.library.servo;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


// basic servo with no set positions at all
public class Servob {
    private final String servoName;
    protected Servo servo;

    public Servob(String servoName){
        this.servoName = servoName;

    }


    public void setServo(HardwareMap map){
        this.servo = map.get(Servo.class, servoName);
    }

    public Servo getServo(){
        return servo;
    }

    public double getPos(){
        return servo.getPosition();
    }

    public void setPos(double pos){
        servo.setPosition(pos);
    }

}
