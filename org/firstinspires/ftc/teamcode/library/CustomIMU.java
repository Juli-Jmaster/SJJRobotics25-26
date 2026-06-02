package org.firstinspires.ftc.teamcode.library;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.teamcode.library.Utils;

// class for handling the IMU and its calculations
//because of how the robot gets its data in -180 to 180
//the defalt straight is 180 becasue we mad it so now the values are between 0 and 360
//just added +180 to original number
//thats why the new straight ahead is 180
public class CustomIMU {
    private final String name;
    private double rotationLeft = 0;
    private IMU imu;
//    private double angleDifference = 0;

    public CustomIMU(String name){
        this.name = name;
    }


    public void setImu(HardwareMap map, ImuOrientationOnRobot imuOrientationOnRobot){
        this.imu = map.get(IMU.class, name);
        IMU.Parameters parameters = new IMU.Parameters(imuOrientationOnRobot);
        imu.initialize(parameters);
//        imu.resetYaw();
    }
    public IMU getImu() {
        return imu;
    }

    public void resetYaw(){
        imu.resetYaw();
    }

    // so now 180 is straight ahead and directly behind is 0 or 360
    public double getYaw(){
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    //get if it not facing the target with a tolerance within a time limit
    //broken timer
    public boolean notFacingTimer(double target, double start, ElapsedTime runtime, int timeInSec){
        if(runtime.seconds() > timeInSec){
            return false;
        } else {
            return notFacing(target, start);
        }
    }

    //get if it not facing the target with a tolerance
    public double getRotationLeftPower(int target/*, Telemetry telemetry*/){
        double rX =0;
        double currentYaw = getYaw();

        double angleDifference = target - currentYaw;

        // Normalize target to [-180, 180]
        if (angleDifference > 180) {
            angleDifference -= 360;
        }

        if (angleDifference < -180) {
            angleDifference += 360;
        }

        if (angleDifference > 0) {
            rX = MovementCurves.quadraticCurve(angleDifference / 360);
        }

        if (angleDifference < 0) {
            rX = -MovementCurves.quadraticCurve(-angleDifference / 360);
        }


        if (rX > 0 && rX < RobotConstants.minimumPowerToTurn) {
            rX = RobotConstants.minimumPowerToTurn;
        }
        if (rX < 0 && rX > -RobotConstants.minimumPowerToTurn) {
            rX = -RobotConstants.minimumPowerToTurn;
        }

        //  telemetry.addData("target", target);
        //   telemetry.addData("current", currentYaw);
        //    telemetry.addData("power", rX);
        //    telemetry.addData("angleDifference", angleDifference);
        //    telemetry.update();

        return rX;
    }

    //returns the power to the left side wheel to turn to correct
    public boolean notFacing(double target, double start) {
        double currentYaw = getYaw();

        return Utils.notFacing(start, currentYaw, target, 3);
    }

}
