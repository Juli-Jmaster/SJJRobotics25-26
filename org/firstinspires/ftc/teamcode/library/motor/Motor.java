package org.firstinspires.ftc.teamcode.library.motor;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

// a class for handling A DcMotor and it's simplified functions for it
public class Motor {
    private String motorname;
    private DcMotor motor;
    private boolean isReversed;
    private boolean encoders;
    private boolean brake;

    public Motor(String name, boolean reversed, boolean brake, boolean encoders){
        this.motorname = name;
        this.isReversed = reversed;
        this.encoders = encoders;
        this.brake = brake;
    }
    //setup for when init hit in auto
    public void setupMotor(HardwareMap map){
        this.motor = map.get(DcMotor.class, motorname);
        loadMotor();
    }

    private void loadMotor() {
        if(isReversed){motor.setDirection(DcMotor.Direction.REVERSE);}
        if(encoders){setMode(DcMotor.RunMode.RUN_USING_ENCODER);}
        else {setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);}
        if(brake){motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);}
    }

    public DcMotor getMotor() {
        return motor;
    }
    private void setMode(DcMotor.RunMode mode) {
        motor.setMode(mode);
    }


    //sets its target and tell robot to move
    public void move(int ticks, double power) {
        motor.setTargetPosition(ticks);
        setPower(power);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    //does nto set any power to start with
    public void move(int ticks) {
       this.move(ticks, 0.0);
    }

    //TODO: TEST (replace reached with isBusy)
    public void moveIf(){
        if (!motor.isBusy() && !brake) {
            stopMotor();
        }
    }
    //? remove?
    public  void moveWhile() {
        if (!motor.isBusy() && !brake) {
            stopMotor();
        }
    }


    public void setPower(double power) {
        motor.setPower(power);
    }
    public void stopMotor() {
        motor.setPower(0);
    }
    public void reset(){
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        loadMotor();
    }
}
