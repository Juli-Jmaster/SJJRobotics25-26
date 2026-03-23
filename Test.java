
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.library.CustomIMU;
import org.firstinspires.ftc.teamcode.library.MovementCurves;
import org.firstinspires.ftc.teamcode.library.RobotConstants;


@TeleOp

public class Test extends LinearOpMode {
    private final CustomIMU imu = new CustomIMU("imu");

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor frontLeft;
    private DcMotor frontRight;

    @Override
    public void runOpMode() {
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        // frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        imu.setImu(hardwareMap, new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double rl = imu.getRotationLeftPower(100);
            while (imu.notFacingTimer(100, runtime, RobotConstants.defaultTimeWaitForTurn)) {
                rl = imu.getRotationLeftPower(100);
                drive((float) rl);
            }
            telemetry.addData("f", imu.getImu().getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.update();
            drive(0);
//            telemetry.addData("Status", "Running");
//            telemetry.update();

        }
    }

    public void drive(float y) {

        backLeft.setPower(y); //backR
        backRight.setPower(-y); //frontL
        frontLeft.setPower(y);  //frontR
        frontRight.setPower(-y);


    }

    private double RangeLimit(float x, float y, float rx, double value) {
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);


        return (value / denominator);
    }

}
