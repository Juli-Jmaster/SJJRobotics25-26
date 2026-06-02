package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import java.util.ArrayList;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import java.util.List;

@TeleOp

public class DriveTrigger extends LinearOpMode {

    //drive
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotorEx outtake;
    private DcMotor intake1;
    private DcMotor intake2;
    private Servo hood;
    private boolean oncea = false;
    private boolean onceb = false;
    private boolean shoot = false;
    private int inc = 900;//1250

    //limelight
    private Limelight3A limelight;
    private IMU imu;
    private double error = 0;
    double lastError = 0;
    private double angleTolerance;
    private double goalX = 0;
    private double kP = 0.04;
    private double kD = 0.007;
    private double curTime = 0;
    private double lastTime;
    private double rotate=0;
    private double hoodA=0;
    private List<Double> list = new ArrayList<>();


    @Override
    public void runOpMode() throws InterruptedException {
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        outtake = hardwareMap.get(DcMotorEx.class, "outtake");
        intake1 = hardwareMap.get(DcMotor.class, "intake1");
        intake2 = hardwareMap.get(DcMotor.class, " intake2");
        hood =  hardwareMap.get(Servo.class, "hood");
//        kick = hardwareMap.get(Servo.class, "kick");
//        kick.setPosition(0.8);



        // frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        //  outtake.setDirection(DcMotorSimple.Direction.REVERSE);
        intake1.setDirection(DcMotorSimple.Direction.REVERSE);
//      SIXXTYY SEEVEEENN LOLOLOLOLOLOL TRALALALALLAAAAAAA 67 GEETTT OOUUUTTTT!!!!
        intake2.setDirection(DcMotorSimple.Direction.REVERSE);

        outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtake.setVelocityPIDFCoefficients(
                100,   // P
                0, // I
                0.,    // D
                18.45  // F
        );
        intake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        frontLeft.setPower(1);
        limelightInit();
        waitForStart();
        hood.setPosition(.49);
        hoodA=.49;
        limelightStart();
        while (opModeIsActive()){
            drive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
            if (gamepad1.a){
                hood.setPosition(.48);
                inc=90;
                //.55 for close to goal 3900rpm
                //.65 for far to goal 3300rpm
            } else if (gamepad1.b){
                inc=1250;
                hood.setPosition(.37);
                //.55 for close to goal 3900rpm
                //.65 for far to goal 3300rpm
            }

            if (gamepad1.x){
                hoodA+=0.001;
                hood.setPosition(hoodA);
            } else if(gamepad1.y) {
                hoodA-=0.001;
                hood.setPosition(hoodA);
            }


            if (gamepad1.left_trigger >= 0.2 && ((outtake.getVelocity() >= inc - 40 && outtake.getVelocity() <= inc + 40) || shoot==true)) {
                // list.add(outtake.getVelocity());
                shoot=true;
//                transfer = !transfer;
                // if(outtake.getVelocity() >= 880){
                intake1.setPower(.9);
                intake2.setPower(1);
                // outtake.setPower(1);
                // }
                outtake.setVelocity(2500);


            } else if(gamepad1.right_trigger >= 0.2){
                shoot=false;
                intake2.setPower(1);
            } else {
                intake1.setPower(0);
                intake2.setPower(0);
//              outtake.setPower(1);
                outtake.setVelocity(inc);
            }




            if(gamepad1.start && !(oncea)){
                inc+=1;
                oncea =true;
            } else {
                oncea =false;
            }

            if(gamepad1.back && !(onceb)){
                inc-=1;
                onceb =true;
            } else {
                onceb =false;
            }

            if(gamepad1.dpad_right){
                drive((float)0.75,0,0);
            } else if(gamepad1.dpad_left){
                drive((float)-0.75,0,0);
            } else if(gamepad1.dpad_up){
                drive(0,(float)-0.75,0);
            } else if(gamepad1.dpad_down){
                drive(0,(float)0.75,0);
            }

            if(gamepad1.left_bumper){
                limelight();
            }
            lastError = 0;
            lastTime = getRuntime();
            rotate=0;

            // telemetry.update();
            // backRight.setPower(1);
            //long before time had a name, the first spinjitsu master created ninjago GET OUT

            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                telemetry.addData("Ta", result.getTa());
                telemetry.addData("distance", distanceEQ(result.getTa()));

            }
            telemetry.addData("setVel", inc);
            telemetry.addData("vel", outtake.getVelocity());
            telemetry.addData("position", hood.getPosition());
            // telemetry.addData("list", list);
            telemetry.update();
        }

    }

    private void limelight() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());
        LLResult result = limelight.getLatestResult();
        if (result != null) {
            if (result.isValid()) {
                double distance = distanceEQ(result.getTa());
                //hood adjust
                hood.setPosition(hoodEQ(distance));
                //velocity adjust
                inc=(int)velcoityEQ(distance);
                outtake.setVelocity(inc);


                //addjust to the goal
                curTime = getRuntime();
                error = goalX - result.getTx();

                if (Math.abs(error) > 3) {
                    kP = 0.04;
                    kD = 0.007;
                } else {
                    kP = 0.11;
                    kD = 0.004;
                }

                double pTerm = error * kP;
                double dT = Math.max(curTime - lastTime, 0.001);
                double dTerm = ((error - lastError) / dT) * kD;

                rotate = Range.clip(pTerm + dTerm, -0.4, 0.4);
                if (Math.abs(error) < 0.5) {
                    rotate = 0;
                } else {
                    rotate*=-1;
                }

                lastError = error;
                lastTime = curTime;
            }
        } else {
            lastError = 0;
            lastTime = getRuntime();
            rotate=0;
        }

        drive((float) 0, (float) 0, (float) rotate);
    }

    public void limelightInit(){
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP)));
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1);
        waitForStart();
    }
    public void limelightStart(){
        limelight.start();
        imu.resetYaw();
        curTime = getRuntime();
        lastTime = getRuntime();
    }


    public void drive(float x, float y, float rx){
        y=-y;
        x=-x;
//        float y=-gamepad1.left_stick_x;
//        float x=-gamepad1.left_stick_y;
//        float rx=gamepad1.right_stick_x;

        backLeft.setPower(RangeLimit(x,y,rx,y+x+rx)); //backR
        backRight.setPower(RangeLimit(x,y,rx,y-x-rx)); //frontL
        frontLeft.setPower(RangeLimit(x,y,rx,y-x+rx));  //frontR
        frontRight.setPower(RangeLimit(x,y,rx,y+x-rx));


        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private double RangeLimit(float x,float y, float rx,double value){
        double denominator = Math.max(Math.abs(y) + Math.abs(x)+ Math.abs(rx), 1);
        return (value /  denominator)*.925;
    }
    public double distanceEQ(double ta){
        return 65.1194*Math.pow(ta,-0.539833);
    }

    public double hoodEQ(double distance) {
        return 0.2973814 + (0.8807468 - 0.2973814)/(1 + Math.pow(distance/18.23198, 1.423881));
    }
    public double velcoityEQ(double distance){
        return 98191690 + (558.5428 - 98191690)/(1 + Math.pow(distance/4216956, 1.148414));
    }

    // todo: write your code here
}