package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.library.MovementCurves;
import org.firstinspires.ftc.teamcode.library.RobotConstants;
import org.firstinspires.ftc.teamcode.library.RobotStates;
import org.firstinspires.ftc.teamcode.library.drive.DriveMainAuto;



@TeleOp
public class NineCloseRed extends LinearOpMode implements DriveMainAuto {

    private DcMotorEx outtake;
    private DcMotor intake1;
    private DcMotor transfer;
    private boolean got = false;
    private boolean got1 = false;

    @Override
    public void runOpMode() throws InterruptedException {
        loadDrive(hardwareMap, new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP));

        outtake = hardwareMap.get(DcMotorEx.class, "outtake");
        transfer = hardwareMap.get(DcMotor.class, "intake1");
        intake1 = hardwareMap.get(DcMotor.class, "intake2");

        intake1.setDirection(DcMotorSimple.Direction.REVERSE);
        transfer.setDirection(DcMotorSimple.Direction.REVERSE);

        outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtake.setVelocityPIDFCoefficients(
                3.2,   // P
                0.0,   // I
                1,   // D
                13.8   // F
        );
        intake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        imu.resetYaw();
        pinPointOdo.update();
//        outtake.setPower(.75);
        // outtake.setVelocity(1300);
        // while (opModeIsActive()){
        //     powerSidewaysDriveMotors(0.2, RobotStates.y);
        // }

        straightStart(-54.5);
        face(-5);
//        outtake.setPower(0);
        outtake.setVelocity(1300);
        // shootNextWait();
        transferShoot();
        //transfer and shoot sequence
        face(-47);
        sideways(1);
        // //run intake
        straightIntake(44);
        // //stop intake
        // waitSeconds(1);
        intake1.setPower(0);
        // //stop intake
        straight(-38);
        face(0);
        transferShoot();

        face(-47);
        sideways(20); //*******

        straightIntake(47);
        // waitSeconds(1);
        intake1.setPower(0);
        straight(-40);
        sideways(-16);
        face(0);
        transferShoot();
        sideways(20);

        //  sideways(-21);
        //  face(0);
        //  transferShoot();
        //  sideways(20);
        // straight(35);
        // //stop intake
        // straight(-32);
        // sideways(-22);
        // face(0);
        // //shoot sequence
        // sideways(20);

        while (opModeIsActive()){
            telemetry.addData("done: ", "done");
            telemetry.addData("d: ", imu.getYaw());
            telemetry.addData("done11: ", outtake.getVelocity());
            telemetry.update();
        }
    }

    public void shootNextWait(){
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (opModeIsActive()) {
            if (outtake.getVelocity() >= 1220 && outtake.getVelocity() <= 1340) {
                break;
            }
        }
    }

    public void transferShoot(){
        //shootNextWait();
        transfer.setPower(0.8);
        intake1.setPower(0.8);
        waitSeconds(.15);
        transfer.setPower(0);
        intake1.setPower(0);
        shootNextWait();
        // waitSeconds(.5);
        transfer.setPower(0.8);
        intake1.setPower(0.8);
        waitSeconds(.15);
        transfer.setPower(0);
        intake1.setPower(0);
        shootNextWait();
        // waitSeconds(.5);
        transfer.setPower(0.8);
        intake1.setPower(1);
        waitSeconds(.2);
        transfer.setPower(0);


    }
    public void waitSeconds(double seconds) {
        ElapsedTime timer1 = new ElapsedTime();
        timer1.reset();
        while (timer1.seconds() < seconds) {
            // Optional: Idle to let other processes run
            Thread.yield();
        }
    }
    public void straightStart(double inches){
        pinPointOdo.update();
        double start = pinPointOdo.pinPointDrive.getEncoderY();

        //init
        int target =  pinPointOdo.pinPointDrive.getEncoderY()-pinPointOdo.getTicks(inches);
        //inBusy
        double cur = pinPointOdo.pinPointDrive.getEncoderY();
        while(isBusy(target,(int) start, (int) cur)){
            pinPointOdo.update();
            cur = pinPointOdo.pinPointDrive.getEncoderY();

            //nromalize for movementcurve
            double normalizedDistanceTravled = (double)  Math.abs(cur - start) / Math.abs(target - start);
            double power = MovementCurves.movementCurves(RobotConstants.defaultStraightMovementCurve,normalizedDistanceTravled,
                    RobotConstants.defaultStraightMultipier, RobotConstants.minimumPowerStraight*2);
            double totalMinusTravled =   Math.abs(target - start) - Math.abs(cur - start);

            if (totalMinusTravled < pinPointOdo.getTicks(inchesForMinPower)) {
                power = RobotConstants.minimumPowerStraight;
            } else if (totalMinusTravled < pinPointOdo.getTicks(inchesForTwiceMinPower)) {
                power = RobotConstants.minimumPowerStraight*2;
            }
            if(outtake.getVelocity() <= 1180){
                outtake.setPower(1);
            } else {
                outtake.setPower(0);
                outtake.setVelocity(1300);
            }

            //set power from movementcurve
            powerStraightDriveMotors(power*Math.signum(inches), RobotStates.yaw);

        }
        //stop when done
        stopDriveMotors();
    }

    public void straightIntake(double inches){
        pinPointOdo.update();
        double start = pinPointOdo.pinPointDrive.getEncoderY();

        //init
        int target =  pinPointOdo.pinPointDrive.getEncoderY()-pinPointOdo.getTicks(inches);
        //inBusy
        double cur = pinPointOdo.pinPointDrive.getEncoderY();
        while(isBusy(target,(int) start, (int) cur)){
            pinPointOdo.update();
            cur = pinPointOdo.pinPointDrive.getEncoderY();

            //nromalize for movementcurve
            double normalizedDistanceTravled = (double)  Math.abs(cur - start) / Math.abs(target - start);
            double power = MovementCurves.movementCurves(RobotConstants.defaultStraightMovementCurve,normalizedDistanceTravled,
                    0.7, RobotConstants.minimumPowerStraight*2);
            double totalMinusTravled =   Math.abs(target - start) - Math.abs(cur - start);

            if (totalMinusTravled < pinPointOdo.getTicks(inchesForMinPower)) {
                power = RobotConstants.minimumPowerStraight;
            } else if (totalMinusTravled < pinPointOdo.getTicks(inchesForTwiceMinPower)) {
                power = RobotConstants.minimumPowerStraight*2;
            }

            //set power from movementcurve
            powerStraightDriveMotors(power*Math.signum(inches), RobotStates.yaw);

        }
        //stop when done
        stopDriveMotors();
    }
}
