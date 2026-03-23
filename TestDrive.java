package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.library.MovementCurves;
import org.firstinspires.ftc.teamcode.library.RobotConstants;
import org.firstinspires.ftc.teamcode.library.RobotStates;
import org.firstinspires.ftc.teamcode.library.Utils;
import org.firstinspires.ftc.teamcode.library.drive.DriveMainAuto;
import org.firstinspires.ftc.teamcode.library.motor.PinPoint4barOdom;

import static org.firstinspires.ftc.teamcode.library.RobotStates.odo;

@TeleOp
public class TestDrive extends LinearOpMode implements DriveMainAuto {

    PinPoint4barOdom odoNum = new PinPoint4barOdom("odo");


    @Override
    public void runOpMode() throws InterruptedException {

        loadDrive(hardwareMap, new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP));
        int targe1t = new PinPoint4barOdom("yu").getTicks(5);
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Device Version Number:", odo.getDeviceVersion());
        telemetry.addData("Dfgdfgsd", targe1t);
        telemetry.update();
        frontLeftDrive.getMotor().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.getMotor().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.getMotor().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.getMotor().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//        frontLeftDrive.getMotor().setDirection(DcMotorSimple.Direction.REVERSE);
//        frontRightDrive.getMotor().setDirection(DcMotorSimple.Direction.FORWARD);
//        backLeftDrive.getMotor().setDirection(DcMotorSimple.Direction.REVERSE);
//        backRightDrive.getMotor().setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        imu.resetYaw();
        odo.update();
        double start = odo.getEncoderY();
        PinPoint4barOdom odoNum = new PinPoint4barOdom("odo");
        //  turnTo(90);
//        //init
        int target =  (int) odo.getEncoderY()+odoNum.getTicks(5);
//        //inBusy
        double cur = odo.getEncoderY();
//
        // while (opModeIsActive()){
        //     odo.update();
        //     cur = odo.getEncoderY();
        //     telemetry.addData("target: ", target);
        //     telemetry.addData("start: ", start);
        //     telemetry.addData("curr: ", cur);
        //     telemetry.addData("cur left: ",(double)  Math.abs(cur - start));
        //     telemetry.addData("target: ",(double)  Math.abs(target - start));
        //     telemetry.addData("total - travled: ",(double)   Math.abs(target - start) - Math.abs(cur - start) );
        //     telemetry.addData("power const: ",(double)  Math.abs(cur - start) / Math.abs(target - start));
        //     telemetry.addData("isBusy: ", isBusy(target,(int) start, (int) cur));
        //     telemetry.update();
        // }


        // while (opModeIsActive()) {
        //     drive(0.09, RobotStates.yaw);
        // }

        straight(5);
        // straight(-55);\
        // double start1 = imu.getYaw();
        // double currr = imu.getYaw();
        // while(Utils.notFacing(start1, currr, 90, 3)){
        //     currr=imu.getYaw();
        //     drive(0,90);
        // }
        // stopDriveMotors();

        sideways(-15);
        turnTo(90);
//        RobotStates.y +=target;
        while (opModeIsActive()){
            double yaw = imu .getYaw();
            double error = AngleUnit.normalizeDegrees(yaw - 90);

            telemetry.addData("Yaw", yaw);
            telemetry.addData("Target", 90);
            telemetry.addData("Error", error);
            telemetry.addData("Not Facing", Math.abs(error) > RobotConstants.defaultToleranceFace);
            // telemetry.update();

            // odo.update();
            // double currentYaw = imu.getYaw();

            // telemetry.addData("Status", odo.getDeviceStatus());
            // telemetry.addData("ye1s", odo.getEncoderY());
            // telemetry.addData("tar", target);
            // telemetry.addData("start", imu.getYaw());

            // telemetry.addData("ye221s",isBusy(target, (int)start, odo.getEncoderY()));

            telemetry.update();
        }


//        telemetry.addData("drive", "frontLeftDrive");
//        telemetry.update();
//        frontLeftDrive.setPower(0.4);
//        waitMe(3);
//        frontLeftDrive.setPower(0);
//
//
//        telemetry.addData("drive", "frontRightDrive");
//        telemetry.update();
//        frontRightDrive.setPower(0.4);
//        waitMe(3);
//        frontRightDrive.setPower(0);
//
//        telemetry.addData("drive", "backLeftDrive");
//        telemetry.update();
//        backLeftDrive.setPower(0.4);
//        waitMe(3);
//        backLeftDrive.setPower(0);
//
//        telemetry.addData("drive", "backRightDrive");
//        telemetry.update();
//        backRightDrive.setPower(0.4);
//        waitMe(3);
//        backRightDrive.setPower(0);
//
//        waitMe(3);
//        frontRightDrive.setPower(0.4);
//        frontLeftDrive.setPower(0.4);
//        backLeftDrive.setPower(0.4);
//        backRightDrive.setPower(0.4);
//        waitMe(3);
//        frontRightDrive.setPower(0);
//        frontLeftDrive.setPower(0);
//        backLeftDrive.setPower(0);
//        backRightDrive.setPower(0);

    }
    public void waitSeconds(double seconds) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        while (timer.seconds() < seconds) {
            // Optional: Idle to let other processes run
            Thread.yield();
        }
    }

    public void straight(int inches){
        odo.update();
        double start = odo.getEncoderY();

        //init
        int target =  (int) odo.getEncoderY()-odoNum.getTicks(inches);
        //inBusy
        double cur = odo.getEncoderY();
        while(isBusy(target,(int) start, (int) cur)){
            odo.update();
            cur = odo.getEncoderY();

            //nromalize for movementcurve
            double normalizedDistanceTravled = (double)  Math.abs(cur - start) / Math.abs(target - start);
            double power = MovementCurves.movementCurves(RobotConstants.defaultDriveMovementCurve,normalizedDistanceTravled,
                    1);
            double totalMinusTravled =   Math.abs(target - start) - Math.abs(cur - start);

            if (totalMinusTravled < odoNum.getTicks(2)) {
                power = .1;
            } else if (totalMinusTravled < odoNum.getTicks(6)) {
                power = .2;
            }

            //set power from movementcurve
            drive(power*Math.signum(inches),0, RobotStates.yaw);

        }
        //stop when done
        stopDriveMotors();
    }


    public void sideways(int inches){
        odo.update();
        double start = odo.getEncoderX();

        //init
        int target =  (int) odo.getEncoderX()+odoNum.getTicks(inches);
        //inBusy
        double cur = odo.getEncoderX();
        while(isBusy(target,(int) start, (int) cur)){
            odo.update();
            cur = odo.getEncoderX();

            //nromalize for movementcurve
            double normalizedDistanceTravled = (double)  Math.abs(cur - start) / Math.abs(target - start);
            double power = MovementCurves.movementCurves(RobotConstants.defaultStraightMovementCurve,normalizedDistanceTravled,
                    1);
            double totalMinusTravled =   Math.abs(target - start) - Math.abs(cur - start);

            if (totalMinusTravled < odoNum.getTicks(2)) {
                power = .15;
            } else if (totalMinusTravled < odoNum.getTicks(6)) {
                power = .2;
            }

            //set power from movementcurve
            drive(0,power*Math.signum(inches), 0);

        }
        //stop when done
        stopDriveMotors();
    }


    public void drive(double x, double y, int h){
        double rl = imu.getRotationLeftPower(h) *0.5;
//        telemetry.addData("rl", rl);
//        telemetry.update();
        frontRightDrive.setPower(x - y + rl);
        backRightDrive.setPower(x +y + rl);
        frontLeftDrive.setPower(x +y - rl);
        backLeftDrive.setPower(x -y - rl);
    }
}