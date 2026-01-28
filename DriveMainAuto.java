package org.firstinspires.ftc.teamcode.library.drive;

import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.library.RobotConstants;
import org.firstinspires.ftc.teamcode.library.RobotState;

@FunctionalInterface
interface Equation {
    double apply(double x);
};


public interface DriveMainAuto extends MotorUtils {

    ElapsedTime runtime = new ElapsedTime();

    default void loadDrive(HardwareMap hardwareMap, ImuOrientationOnRobot imuOrientationOnRobot){
        //load imu
        imu.setImu(hardwareMap, imuOrientationOnRobot);
        RobotState.yaw = 0;

        //load drive motors
        startUsingMotors(hardwareMap);

        //load odometer Motors
//        straight.setMotor(hardwareMap.get(DcMotorEx.class, straight.motorname));
//        sideways.setMotor(hardwareMap.get(DcMotorEx.class, "sideways"));
//        //resets the position of odmometry motors
//        sideways.getMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        straight.getMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    //simplified movement for the motors
    default void forward(double inches){
    }
    default void backward(double inches){
    }
    default void right(double inches){
    }
    default void left(int inches){
    }

    default void straight(double inches, int flip, int straightFacing){

    }

    default void sidways(double inches, int flip, int straightFacing){

    }


    default void movementCustomArc(double inchesForward, double inchesLeft, Equation leftEqu, int straightFacing){
        int flipForward = (int) Math.signum(inchesForward);
        int flipLeft = (int) Math.signum(inchesLeft);
        double y = leftEqu.apply(0.2) *  flipLeft;

    }


    default void turnTo(int degree){
        runtime.reset();
        while(imu.notFacingTimer(degree, runtime, RobotConstants.defaultTimeWaitForTurn)){
            powerStraightDriveMotors(0.0, degree);
        }
        RobotState.yaw = degree;
        stopDriveMotors();
    }
}
