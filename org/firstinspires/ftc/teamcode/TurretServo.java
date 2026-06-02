package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.Range;

@TeleOp

public class TurretServo extends LinearOpMode {

    private Servo t2;
    private Servo t1;
    private double adjust = 0.00125;
    public boolean block = false;
    //block the search feature
    public boolean blockss = false;

    //limelightTurret
    private Limelight3A limelight;
    private double error = 0;
    double lastError = 0;
    private double goalX = 0;
    private double kP = 0.04;
    private double kD = 0.007;
    private double curTime = 0;
    private double lastTime;
    private double lastPos;
    private double rotate=0;
    private double notRotateTxRange = 5;
    private double kPDSwitch = 5;

    @Override
    public void runOpMode() {

        t1 =  hardwareMap.get(Servo.class, "t1");
        t2 =  hardwareMap.get(Servo.class, "t2");
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        limelightInit();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        limelightStart();
        t1.setPosition(.5);
        t2.setPosition(.5);
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if (gamepad1.y){
                if(blockss){
                    blockss=false;
                } else if(!blockss){
                    blockss=true;
                }
            }
            if(gamepad1.x){
                if(block){
                    block=false;
                } else if (!block){
                    block = true;
                }
            }
            limelightTurret();
            telemetry.addData("block", block);
            telemetry.addData("blockss", blockss);
            telemetry.update();
            if (gamepad1.b);
            //t2.setPosition(.2);
            //t1.setPosition(.8);

        }
    }
    public void limelightInit(){
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1);
        waitForStart();
    }
    public void limelightStart(){
        limelight.start();
        curTime = getRuntime();
        lastTime = getRuntime();
        lastPos = 0.5;
    }

    private void limelightTurret() {
        // add limelightTurret with imu
        // YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        // limelightTurret.updateRobotOrientation(orientation.getYaw());

        //get reuslts and chekc if valid
        LLResult result = limelight.getLatestResult();
        if (result != null) {
            if (result.isValid()) {
                //log to DriverHub Tx value
                telemetry.addData("curentx", result.getTx());

                //how far off from target Tx
                curTime = getRuntime();
                error = goalX - result.getTx();

                //kP and kD values for how much by
                //currently using: Servo
                if (Math.abs(error) > kPDSwitch) {
                    //outside kPDSwitch on each side
                    kP = 0.00006;
                    kD = 0;
                } else {
                    //inside kPDSwitch on each side
                    kP = 0;
                    kD = 0;
                }

                //calculate adjustment
                double pTerm = error * kP;
                double dT = Math.max(curTime - lastTime, 0.001);
                double dTerm = ((error - lastError) / dT) * kD;
                //limit the rotate so not too powerfull
                //could not use this and just have a smaller kP and kD
                rotate = Range.clip(pTerm + dTerm, -0.7, 0.7);

                //so does not rotate if within range
                if (Math.abs(error) < notRotateTxRange) {
                    rotate = 0;
                } else if (Math.abs(error) > notRotateTxRange) {
                    rotate*=-1;

                    //get last direction to turret search
                    //method: Servo
                    lastPos=Math.signum(rotate);
                }

                //reset for next time
                lastError = error;
                lastTime = curTime;
                rotateTurret(rotate);
            } else {
                lastError = 0;
                lastTime = getRuntime();
                searchSmall();
            }
        } else {
            lastError = 0;
            lastTime = getRuntime();
            searchSmall();
        }
        telemetry.addData("me", rotate);
    }


    private void rotateTurret(double rotate){
        if(rotate!=0 && !block){
            t1.setPosition(t1.getPosition()+rotate);
            t2.setPosition(t2.getPosition()+rotate);
        }
    }
    //search mode for servo
    //if servo going the same direction when moving for search mode
    private void search(){
        double cur = t1.getPosition();
        //move to left
        if(lastPos > 0 && !blockss){
            t1.setPosition(t1.getPosition()+ adjust);
            t2.setPosition(t2.getPosition()+ adjust);
            if (t1.getPosition() >= 1){
                //switch to go to the right
                lastPos=-1;
            }
            //move to the right
        } else if(lastPos < 0 && !blockss){
            t1.setPosition(t1.getPosition()- adjust);
            t2.setPosition(t2.getPosition()- adjust);
            if (t1.getPosition() <= 0){
                //switch to go to the left
                lastPos=1;
            }
        }
    }

    //search mode for servo
    //if servo going the same direction when moving for search mode
    private void searchSmall(){
        double cur = t1.getPosition();
        //move to left
        if(lastPos > 0 && !blockss){
            t1.setPosition(t1.getPosition()+ adjust);
            t2.setPosition(t2.getPosition()+ adjust);
            if (t1.getPosition() >= .7){
                //switch to go to the right
                lastPos=-1;
            }
            //move to the right
        } else if(lastPos < 0 && !blockss){
            t1.setPosition(t1.getPosition()- adjust);
            t2.setPosition(t2.getPosition()- adjust);
            if (t1.getPosition() <= 0.3){
                //switch to go to the left
                lastPos=1;
            }
        }
    }
}




