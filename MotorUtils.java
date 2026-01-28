package org.firstinspires.ftc.teamcode.library.drive;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.teamcode.library.motor.OdometryMotor;
import org.firstinspires.ftc.teamcode.library.CustomIMU;
import org.firstinspires.ftc.teamcode.library.MovementCurves;
import org.firstinspires.ftc.teamcode.library.RobotConstants;
import org.firstinspires.ftc.teamcode.library.motor.Motor;

// a utils class to load all drive motors and group motor functions
public interface MotorUtils extends RobotConstants {

    //motors for each wheel
    Motor frontRightDrive = new Motor(frontRightMotorName, frontRightReversed, driveMotorBrake, drivemotorEncoders);
    Motor backRightDrive = new Motor(backRightMotorName, backRightReversed, driveMotorBrake, drivemotorEncoders);
    Motor frontLeftDrive = new Motor(frontLeftMotorName, frontLeftReversed, driveMotorBrake, drivemotorEncoders);
    Motor backLeftDrive = new Motor(backLeftMotorName, backLeftReversed, driveMotorBrake, drivemotorEncoders);

    //use of odometry motor and IMU
//    OdometryMotor straight = new OdometryMotor("straight", diameterLengthType, diameterLength, ticksPerType, ticksPerTypeNumber);
//    OdometryMotor sideways = new OdometryMotor("sideways", diameterLengthType, diameterLength, ticksPerType, ticksPerTypeNumber);
    CustomIMU imu = new CustomIMU("imu");

    //used for stopping motors if or while
    //should never use while
    //TODO: TEST (replace reached with isBusy)
    default void driveStopIf(boolean reached) {
        if ((  !frontLeftDrive.getMotor().isBusy() && !frontRightDrive.getMotor().isBusy()
                  && !backLeftDrive.getMotor().isBusy() && !backRightDrive.getMotor().isBusy()   )) {
              stopDriveMotors();
        }
    }

    //forward is positive flip
    default void powerStraightDriveMotors(double x, int target) {
        double rl = imu.getRotationLeftPower(target);
        frontRightDrive.setPower(MovementCurves.movementCurves(defaultDriveMovementCurve, x) - rl);
        backRightDrive.setPower(MovementCurves.movementCurves(defaultDriveMovementCurve, x) - rl);
        frontLeftDrive.setPower(MovementCurves.movementCurves(defaultDriveMovementCurve, x) + rl);
        backLeftDrive.setPower(MovementCurves.movementCurves(defaultDriveMovementCurve, x) + rl);
    }

    //right is positive flip
    default void powerSidewaysDriveMotors(double x, int target) {
        double rl = imu.getRotationLeftPower(target);
        frontRightDrive.setPower(-MovementCurves.movementCurves(defaultDriveMovementCurve, x) - rl);
        backRightDrive.setPower(MovementCurves.movementCurves(defaultDriveMovementCurve, x) - rl);
        frontLeftDrive.setPower(MovementCurves.movementCurves(defaultDriveMovementCurve, x) + rl);
        backLeftDrive.setPower(-MovementCurves.movementCurves(defaultDriveMovementCurve, x) + rl);
    }


    //resetAll the drive motors encoders
    default void cleanupMotors() {
        frontRightDrive.reset();
        frontLeftDrive.reset();
        backLeftDrive.reset();
        backRightDrive.reset();
    }

    default void startUsingMotors(HardwareMap map){
        backLeftDrive.setupMotor(map); //backR
        backRightDrive.setupMotor(map); //frontL
        frontLeftDrive.setupMotor(map);  //frontR
        frontRightDrive.setupMotor(map);
        cleanupMotors();
    }

    //utils for using encoder
    default void stopDriveMotors() {
        frontRightDrive.stopMotor();
        backRightDrive.stopMotor();
        frontLeftDrive.stopMotor();
        backLeftDrive.stopMotor();
    }

    // Computes the current battery voltage
    default double getBatteryVoltage(HardwareMap hardwareMap) {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }

//TODO: will be done in DriveMainAuro class

//    default void setEncoderDriveTarget(int MOVE, int target) {
//        if (MOVE == sideways) {
//            frontRightDrive.moveNoPower(-target);
//            frontLeftDrive.moveNoPower(target);
//            backRightDrive.moveNoPower(target);
//            backLeftDrive.moveNoPower(-target);
//        } else {
//            frontRightDrive.moveNoPower(target);
//            frontLeftDrive.moveNoPower(target);
//            backLeftDrive.moveNoPower(target);
//            backRightDrive.moveNoPower(target);
//        }
//    }
}