package org.firstinspires.ftc.teamcode.library.motor;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class AvgOdomMotor{
    private OdometryMotor[] odometrys;
    
    public AvgOdomMotor(OdometryMotor... items){
        this.odometrys = items;
    }

    public int move(double inches) {
        int num = 0;
        for (OdometryMotor each: odometrys) {
            num+=each.getTicks(inches);
        }
        return num / odometrys.length;
    }

    public void resetAll(){
        for (OdometryMotor each: odometrys) {
            each.getMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    public void setMotors(HardwareMap map){
        for (OdometryMotor each: odometrys) {
            each.setMotor(map.get(DcMotorEx.class, each.motorname));
            each.getMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }
    public int getCurrentPosition() {
        int num = 0;
        for (OdometryMotor each: odometrys) {
            num+=each.getMotor().getCurrentPosition();
        }
        return num / odometrys.length;
    }


}
