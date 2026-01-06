package org.firstinspires.ftc.teamcode.library.motor;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

// a class for handling A DcMotorEx and it's simplified functions for it
public class MotorEx {
    public String motorname;
    protected DcMotorEx motor;

    public MotorEx(String name){
        this.motorname = name;
    }
    //setup for when init hit in auto
    public void setMotor(HardwareMap map) {
        this.motor = map.get(DcMotorEx.class, motorname);
    }
    public void setMotor(DcMotorEx motorg) {
        this.motor = motorg;
    }

    public DcMotorEx getMotor() {
        return motor;
    }
    private void setMode(DcMotor.RunMode mode) {
        motor.setMode(mode);
    }

    //sets its target
    public void move(int ticks) {
        motor.setTargetPosition(ticks);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setPower(double power) {
        motor.setPower(power);
    }

    public void resetPosition() {
        motor.setTargetPosition(motor.getCurrentPosition());
    }
    public void resetPositionToZero(){
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
