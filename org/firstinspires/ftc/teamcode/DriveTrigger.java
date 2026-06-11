package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp

public class DriveTrigger extends LinearOpMode {
    private COLOR color = COLOR.RED;
    private boolean turretMove =false;

    private enum searchSmall {
        NONE, FIRST, SUCCESS
    }
    private searchSmall searchSmallState = searchSmall.NONE;
    private enum turret {
        ADJUST, SEARCH, AT, NONE, SAD
    }
    private turret turretState = turret.NONE;

    private enum search {
        NONE,SMALL
    }
    private search searchState = search.NONE;

    private enum COLOR {
        BLUE, RED;
    }

    //drive
    private ElapsedTime time = new ElapsedTime();
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotorEx outtake;
    private DcMotorEx outtake2;
    private DcMotor transfer;
    private DcMotor intake;
    private Servo hood;
    private Servo t1;
    private Servo t2;
    private boolean oncea = false;
    private boolean onceb = false;
    private boolean shoot = false;
    private int inc = 700;//1250
    private int tarId = 20;

    //limelight
    private Limelight3A limelight;
    private IMU imu;
    private double error = 0;
    private double lastError = 0;
    private double goalX = 2;
    private double kP = 0.04;
    private double kPoutside = 0.04;
    private double kPinside = 0.04;
    private double kD = 0.007;
    private double curTime = 0;
    private double lastTime;
    private double rotate = 0;
    private double hoodA = 0;
    private double adjust = 0.008;
    private double notRotateTxRange = 1.2;
    private double kPDSwitch = 5;
    private double lastPos =0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        backLeft = hardwareMap.get(DcMotor.class, "BL");
        backRight = hardwareMap.get(DcMotorEx.class, "BR");
        frontLeft = hardwareMap.get(DcMotor.class, "FL");
        frontRight = hardwareMap.get(DcMotor.class, "FR");
        outtake = hardwareMap.get(DcMotorEx.class, "shoot");
        outtake2 = hardwareMap.get(DcMotorEx.class, "shoot2");
        transfer = hardwareMap.get(DcMotor.class, "transfer");
        intake = hardwareMap.get(DcMotor.class, " intake");
        hood = hardwareMap.get(Servo.class, "hood");
        t1 = hardwareMap.get(Servo.class, "t1");
        t2 = hardwareMap.get(Servo.class, "t2");
//        kick = hardwareMap.get(Servo.class, "kick");
//        kick.setPosition(0.8);


        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        //intake.setDirection(DcMotorSimple.Direction.FORWARD);
        transfer.setDirection(DcMotorSimple.Direction.REVERSE);

        // outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtake.setVelocityPIDFCoefficients(

                100,   // P
                0., // I
                0,    // D
                15.2  // F
        );
        outtake2.setVelocityPIDFCoefficients(

                100,   // P
                0., // I
                0,    // D
                15.2  // F
        );
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (color==COLOR.BLUE){
            tarId=20;
            goalX=2;
        } else if (color==COLOR.RED){
            tarId=24;
            goalX=-2;
        }

//        frontLeft.setPower(1);
        limelightInit();
        waitForStart();
        hood.setPosition(0.0);
        t1.setPosition(0.5);
        t2.setPosition(0.5);
        // hoodA=.49;
        limelightStart();
        while (opModeIsActive()) {
            drive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
            turret();
            if (gamepad1.ps) {
                inc = 0;
            }
            if (gamepad1.a) {
                hood.setPosition(.48);
                inc = 90;
                //.55 for close to goal 3900rpm
                //.65 for far to goal 3300rpm
            } else if (gamepad1.b) {
                inc = 1250;
                hood.setPosition(.37);
                //.55 for close to goal 3900rpm
                //.65 for far to goal 3300rpm
            }

            if (gamepad1.x) {
                hoodA += 0.001;
                hood.setPosition(hoodA);
            } else if (gamepad1.y) {
                hoodA -= 0.001;
                hood.setPosition(hoodA);
            }
            if(gamepad1.left_trigger >= .2){
                turretMove = true;
            } else {
                turretMove = false;
            }

            if (gamepad1.left_trigger >= .2 && ((outtake.getVelocity() >= inc - 40 && outtake.getVelocity() <= inc + 40) || shoot) && (turretState == turret.AT || shoot)) {
                shoot = true;
                intake.setPower(.8);
                transfer.setPower(.9);
                outtake.setPower(1);
                outtake2.setPower(1);


            } else if (gamepad1.right_trigger >= 0.2) {
                shoot = false;
                transfer.setPower(-.5);
                intake.setPower(0.8);
            } else if(gamepad1.left_bumper){
                turretMove=true;
            }else {
                transfer.setPower(0);
                intake.setPower(0);
                outtake.setVelocity(inc); //inc
                outtake2.setVelocity(inc); //inc

            }
            // if(time.seconds() > .1 &&time.seconds() > .2 && shoot==true){
            //     intake.setPower(.9);
            //     transfer.setPower(1);
            //     outtake.setPower(1);
            // }

            if (gamepad1.start && !(oncea)) {
                gamepad1.runRumbleEffect(new Gamepad.RumbleEffect.Builder().addStep(0.5, 0.5, 500).build());
                inc += 1;
                oncea = true;
            } else {
                oncea = false;
            }

            if (gamepad1.back && !(onceb)) {
                gamepad1.runRumbleEffect(new Gamepad.RumbleEffect.Builder().addStep(0.5, 0.5, 500).build());
                inc -= 1;
                onceb = true;
            } else {
                onceb = false;
            }

            if (gamepad1.dpad_right) {
                drive((float) 0.75, 0, 0);
            } else if (gamepad1.dpad_left) {
                drive((float) -0.75, 0, 0);
            } else if (gamepad1.dpad_up) {
                drive(0, (float) -0.75, 0);
            } else if (gamepad1.dpad_down) {
                drive(0, (float) 0.75, 0);
            }

//            if(gamepad1.left_bumper){
//                limelight();
//            }
            lastError = 0;
            lastTime = getRuntime();
            rotate = 0;

            // telemetry.update();
            // backRight.setPower(1);
            //long before time had a name, the first spinjitsu master created ninjago GET OUT

            telemetry.addData("side:", color);
            telemetry.addData("setVel:", inc);
            telemetry.addData("t1 pos:", t1.getPosition());
            telemetry.addData("hood pos:", hood.getPosition());
            telemetry.addData("outtake vel:", outtake.getVelocity());
            telemetry.addData("outtake2 vel:", outtake2.getVelocity());
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                telemetry.addData("Ta:", result.getTa());
                telemetry.addData("Tx:", result.getTx());
            }
            telemetry.update();
        }

    }

    private boolean limelight() {
        boolean ret = false;
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());
        LLResult result = limelight.getLatestResult();
        if (result != null) {
            if (result.isValid()) {
                //hood adjust
                //hood.setPosition(hoodEQ(result.getTa()));
                //velocity adjust
                //inc=(int)velcoityEQ(result.getTa());
                // outtake.setVelocity(result.getTa());


                //addjust to the goal
                curTime = getRuntime();
                error = goalX - result.getTx();

                if (Math.abs(error) > 4) {
                    kP = 0.045;
                    kD = 0.007;
                } else {
                    kP = 0.065;
                    kD = 0.004;
                }

                double pTerm = error * kP;
                double dT = Math.max(curTime - lastTime, 0.001);
                double dTerm = ((error - lastError) / dT) * kD;


                rotate = Range.clip(pTerm + dTerm, -0.4, 0.4);
                // was -.4 and .4
                if (Math.abs(error) < 2) {
                    ret = true;
                    rotate = 0;
                } else {
                    ret = false;
                    rotate *= -1;
                }

                lastError = error;
                lastTime = curTime;
            }
        } else {
            lastError = 0;
            lastTime = getRuntime();
            rotate = 0;
        }
        drive((float) 0, (float) 0, (float) rotate);
        return ret;
    }

    public void limelightInit() {
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP)));
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1);
        waitForStart();
    }

    public void limelightStart() {
        limelight.start();
        imu.resetYaw();
        curTime = getRuntime();
        lastTime = getRuntime();
    }


    public void drive(float x, float y, float rx) {
        y = -y;
        x = -x;
//        float y=-gamepad1.left_stick_x;
//        float x=-gamepad1.left_stick_y;
//        float rx=gamepad1.right_stick_x;

        backLeft.setPower(RangeLimit(x, y, rx, y + x + rx)); //backR
        backRight.setPower(RangeLimit(x, y, rx, y - x - rx)); //frontL
        frontLeft.setPower(RangeLimit(x, y, rx, y - x + rx));  //frontR
        frontRight.setPower(RangeLimit(x, y, rx, y + x - rx));


        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private double RangeLimit(float x, float y, float rx, double value) {
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        return (value / denominator) * 1;
    }

    public double distanceEQ(double ta) {
        return 65.1194 * Math.pow(ta, -0.539833);
    }

    public double hoodEQclose(double x) {
        // return -199.5927 + (0.5975992 - -199.5927) / (1 + Math.pow((x / 10182830), 0.5112046));
        // NEWER BAD return 0.07887639 + (0.08804108 - 0.07887639)/(1 + Math.pow((x/2.157642),10.62704));
        return 0.04866842 + (0.1010431 - 0.04866842)/(1 + Math.pow((x/3.396128),2.283741));
    }

    public double velocityEQclose(double x) {
        // return 564.2325 + (457187100 - 564.2325) / (1 + Math.pow((x / 0.00001297586), 1.286723));
        //NEWER    return (647.3348 + (916.5553 - 647.3348)/(1 + Math.pow((x/1.241976),1.971656)))-20;
        return (621.4664 + (788.5438 - 621.4664)/(1 + Math.pow((x/1.663893),3.428819)))+20;

    }

    public double hoodEQfar(double x) {
        return  0.08833612 + (3.147761 / Math.pow(2, x / 0.0309554));
    }

    public double velocityEQfar(double x) {
        return 919.3999 + 61280.93*Math.exp(-24.60434*x);
    }



    private void turretAdjust(LLResult result) {
        error = goalX - result.getTx();
        //log to DriverHub Tx value

        //how far off from target Tx
        curTime = getRuntime();
        error = goalX - result.getTx();

        //kP and kD values for how much by
        //currently using: Servo
        if (Math.abs(error) > kPDSwitch) {
            //outside kPDSwitch on each side
            kP = kPoutside;
            kD = 0;
        } else {
            //inside kPDSwitch on each side
            kP = kPinside;
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
            rotate *= -1;

            //get last direction to turret search
            //method: Servo
            if(!(rotate==0)){
                lastPos = Math.signum(rotate);
            }
        }

        //reset for next time
        lastError = error;
        lastTime = curTime;
        rotateTurret(rotate);
    }

    private void turret() {
        turretState = turret.NONE;
        //get reuslts and chekc if valid
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            for (LLResultTypes.FiducialResult fud : result.getFiducialResults()) {
                int id = fud.getFiducialId();
                telemetry.addData("id of tag: ", id);
                if (tarId == id) {
                    error = goalX - result.getTx();

                    //different number for far or close side
                    if (result.getTa() < .4){
                        notRotateTxRange=1.2;
                        //far auto
                        inc = (int) velocityEQfar(result.getTa());
                        hood.setPosition(hoodEQfar(result.getTa()));
                        if (color==COLOR.BLUE){
                            goalX=3;
                        } else if (color==COLOR.RED){
                            goalX=-3;
                        }
                        kPoutside= .00015/1.25;
                        kPinside = 0.0003/1.25;

                    } else if (result.getTa() > .65){
                        notRotateTxRange=3;
                        //close
                        inc = (int) velocityEQclose(result.getTa());
                        hood.setPosition(hoodEQclose(result.getTa()));
                        goalX=0;
                        kPoutside= .00015;
                        kPinside = 0.0003;
                    }
                    if (Math.abs(error) <= notRotateTxRange) {
                        turretState = turret.AT;
                    } else if (Math.abs(error) >= notRotateTxRange) {
                        turretAdjust(result);
                        turretState = turret.ADJUST;
                    }
                } else {
                    lastError = 0;
                    lastTime = getRuntime();
                    turretState = turret.SEARCH;
                    search();
                }
            }
        } else {
            lastError = 0;
            lastTime = getRuntime();
            turretState = turret.SEARCH;
            search();
        }
        if (turretState != turret.SEARCH && turretState != turret.SAD) {
            searchState = search.NONE;
        }
        if (searchState != search.SMALL) {
            searchSmallState = searchSmall.NONE;
        }
        telemetry.addData("state:", turretState);
        telemetry.addData("searchSmallState:", searchSmallState);
        telemetry.addData("searchState:", searchState);

    }
    private void rotateTurret(double rotate) {
        if (rotate != 0 || turretMove) {
            t1.setPosition(t1.getPosition() + rotate);
            t2.setPosition(t2.getPosition() + rotate);
        }
    }

    private void turretSmall() {
        //move to left
        if (lastPos > 0 && !(searchSmallState == searchSmall.SUCCESS))  {
            t1.setPosition(t1.getPosition() + adjust);
            t2.setPosition(t2.getPosition() + adjust);
            if (t1.getPosition() >= .7) {
                //switch to go to the right
                lastPos = -1;
                searchSmallInc();
            }
            //move to the right
        } else if (lastPos < 0 && !(searchSmallState == searchSmall.SUCCESS)) {
            t1.setPosition(t1.getPosition() - adjust);
            t2.setPosition(t2.getPosition() - adjust);
            if (t1.getPosition() <= 0.2) {
                //switch to go to the left
                lastPos = 1;
                searchSmallInc();
            }
        }
    }

    private void searchSmallInc() {
        if (searchSmallState == searchSmall.NONE) {
            searchSmallState = searchSmall.FIRST;
        } else if (searchSmallState == searchSmall.FIRST) {
            searchSmallState = searchSmall.SUCCESS;
        }
    }

    private void restTurret() {
        t1.setPosition(0.5);
        t2.setPosition(0.5);
    }
    private void search(){
        if(turretMove) {
            if (searchSmallState == searchSmall.SUCCESS) {   //searchLargeState == searchLarge.SUCCESS
                restTurret();
                turretState = turret.SAD;
//            searchSmallState = searchSmall.NONE;
            }// else  if (searchSmallState == searchSmall.SUCCESS){
//            searchLargeState = searchLarge.NONE;
//            searchSmallState = searchSmall.NONE;
//            searchState = search.LARGE;
//            turretLarge();
//        }
            if (searchState == search.NONE && turretState == turret.SEARCH) {
                searchState = search.SMALL;
                searchSmallState = searchSmall.NONE;
                turretSmall();
            } else if (searchState == search.SMALL && turretState == turret.SEARCH) {
                turretSmall();
            }// else if (searchState == search.LARGE  && turretState == turret.SEARCH){
//            turretLarge();
//        }

        }
    }
}
// todo: write your code here