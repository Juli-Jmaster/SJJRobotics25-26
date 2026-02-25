//initializing value
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.qualcomm.robotcore.util.Range;

@TeleOp
public class limelight extends LinearOpMode {

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
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private double rotate=0;
    private double cameraAngle=22.45;
    private double aprilTagCenterHeight=29.375;
    private double cameraCenterHeight=14.5;


    @Override
    public void runOpMode() throws InterruptedException
    {
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);


        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP)));
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1);
        waitForStart();

        /*
         * Starts polling for data.
         */
        limelight.start();
        imu.resetYaw();
        curTime = getRuntime();
        lastTime = getRuntime();

        while (opModeIsActive()) {
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            limelight.updateRobotOrientation(orientation.getYaw());
            LLResult result = limelight.getLatestResult();
            if (result != null) {
                if (result.isValid()) {

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
                    double angleToGoalRad = Math.toRadians(cameraAngle + result.getTy());
                    double newtX = Math.toRadians(result.getTx());
                    double distance = ((aprilTagCenterHeight-cameraCenterHeight)/Math.tan(angleToGoalRad));//
                    double newDis = distance/Math.cos(newtX);
                    double errorDis = 48-distance;
                    errorDis*=-1;

                    calibrateMountingAngle(20, result.getTy());

                    //showing telemetry data
                    telemetry.addData("Ty Value", result.getTy());
                    telemetry.addData("Tx Value", result.getTx());
                    telemetry.addData("Ta Value", result.getTa());
                    // telemetry.addData("Distance", newDis);
                    telemetry.addData("newDistance", distanceEQ(result.getTa()));
                    telemetry.addData("Rotate", rotate);
                    telemetry.addData("Distance Error", errorDis);
                    telemetry.update();


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
    }
    public void drive(float x, float y, float rx){

        backLeft.setPower(RangeLimit(x,y,rx,y+x+rx)); //backR
        backRight.setPower(-RangeLimit(x,y,rx,y-x+rx)); //frontL
        frontLeft.setPower(RangeLimit(x,y,rx,y-x-rx));  //frontR
        frontRight.setPower(RangeLimit(x,y,rx,y+x-rx));


        // telemetry.addData("x", gamepad1.left_stick_x);
        // telemetry.addData("y", gamepad1.left_stick_y);
        // frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private double RangeLimit(float x,float y, float rx,double value){
        double denominator = Math.max(Math.abs(y) + Math.abs(x)+ Math.abs(rx), 1);
        // telemetry.addData("x", x);
        // telemetry.addData("y", y);
        // telemetry.addData("rx", rx);
        // telemetry.addData("dem", denominator);
        // telemetry.addData("value", value /  denominator);

        return (value /  denominator)*.75;
    }
    public void calibrateMountingAngle(double measuredDistanceInches, double ty) {

        // finding the hight difference between the limelight and the goal
        double heightDifference = 30 - 14.5;

        // calculating the angle of the limelight
        double calculatedAngle = Math.toDegrees(Math.atan2(heightDifference, measuredDistanceInches)) - ty;

        telemetry.addData("LimeLight Angle", calculatedAngle);
    }
    public double distanceEQ(double ta){
        return 65.1194*Math.pow(ta,-0.539833);
    }

}