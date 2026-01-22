package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class TestDrive extends LinearOpMode {

    private ElapsedTime runtime;
    private DcMotor frontLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backLeftDrive;
    private DcMotor backRightDrive;


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "init");

        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRight");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeft");
        backRightDrive = hardwareMap.get(DcMotor.class, "backRight");

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        runtime = new ElapsedTime();
        runtime.reset();
        telemetry.update();
        waitForStart();

        telemetry.addData("drive", "frontLeftDrive");
        telemetry.update();
        frontLeftDrive.setPower(0.4);
        waitMe(3);
        frontLeftDrive.setPower(0);


        telemetry.addData("drive", "frontRightDrive");
        telemetry.update();
        frontRightDrive.setPower(0.4);
        waitMe(3);
        frontRightDrive.setPower(0);

        telemetry.addData("drive", "backLeftDrive");
        telemetry.update();
        backLeftDrive.setPower(0.4);
        waitMe(3);
        backLeftDrive.setPower(0);

        telemetry.addData("drive", "backRightDrive");
        telemetry.update();
        backRightDrive.setPower(0.4);
        waitMe(3);
        backRightDrive.setPower(0);

        waitMe(3);
        frontRightDrive.setPower(0.4);
        frontLeftDrive.setPower(0.4);
        backLeftDrive.setPower(0.4);
        backRightDrive.setPower(0.4);
        waitMe(3);
        frontRightDrive.setPower(0);
        frontLeftDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);

    }
    private void waitMe(double sec){
        runtime.reset();
        while (runtime.seconds() < sec) {
        }
    }
}