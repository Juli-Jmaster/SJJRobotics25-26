package org.firstinspires.ftc.teamcode.library.servo;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


// basic servo with no set positions at all
public class ContinousServo {
    private final String servoName;
    protected CRServo servo;
    public enum DIRECTION{
        REVERSE,
        FORWARD
    }

    public ContinousServo(String servoName){
        this.servoName = servoName;
    }

    public void setServo(HardwareMap map){
        this.servo = map.get(CRServo.class, servoName);
    }

    public CRServo getServo(){
        return servo;
    }

    public double getPower(){
        return servo.getPower();
    }
    public void setDirection(DIRECTION dir, double power){
        if(dir==DIRECTION.FORWARD){
            servo.setDirection(DcMotorSimple.Direction.FORWARD);
            servo.setPower(power);
        } else if (dir==DIRECTION.REVERSE){
            servo.setDirection(DcMotorSimple.Direction.REVERSE);
            servo.setPower(power);
        }
    }
    public void setDirection(DIRECTION dir){
       setDirection(dir, 0.75);
    }

}
