package org.firstinspires.ftc.teamcode.library.drive;

import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.library.MovementCurves;
import org.firstinspires.ftc.teamcode.library.RobotConstants;
import org.firstinspires.ftc.teamcode.library.RobotStates;


public interface DriveMainAuto extends MotorUtils {

    ElapsedTime runtime = new ElapsedTime();


    default void loadDrive(HardwareMap hardwareMap, ImuOrientationOnRobot imuOrientationOnRobot){
        //load imu
        imu.setImu(hardwareMap, imuOrientationOnRobot);
        RobotStates.yaw = 0;

        pinPointOdo.set(hardwareMap);
        pinPointOdo.pinPointDrive.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinPointOdo.pinPointDrive.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinPointOdo.pinPointDrive.resetPosAndIMU();
        RobotStates.x = 0;
        RobotStates.y = 0;

        //load drive motors
        startUsingMotors(hardwareMap);

        //load odometer Motors
        //     RobotConstants.sidewaysOdometry.setMotors(hardwareMap);
        //   RobotConstants.sidewaysOdometry.setMotors(hardwareMap);


    }

    //simplified movement for the motors
    default void forward(double inches){
    }
    default void backward(double inches){
    }
    default void right(double inches){
    }
    default void left(int inches){}


    default void straight(double inches){
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

            //set power from movementcurve
            powerStraightDriveMotors(power*Math.signum(inches), RobotStates.yaw);

        }
        //stop when done
        stopDriveMotors();
    }

    default void sideways(double inches){
        pinPointOdo.update();
        double start = pinPointOdo.pinPointDrive.getEncoderX();

        //init
        int target = pinPointOdo.pinPointDrive.getEncoderX()+pinPointOdo.getTicks(inches);
        //inBusy
        double cur = pinPointOdo.pinPointDrive.getEncoderX();
        while(isBusy(target,(int) start, (int) cur)){
            pinPointOdo.update();
            cur = pinPointOdo.pinPointDrive.getEncoderX();

            //nromalize for movementcurve
            double normalizedDistanceTravled = (double)  Math.abs(cur - start) / Math.abs(target - start);
            double power = MovementCurves.movementCurves(RobotConstants.defaultSidewaysMovementCurve,normalizedDistanceTravled,
                    RobotConstants.defaultSidewaysMultipier, RobotConstants.minimumPowerSideways*2);
            double totalMinusTravled =   Math.abs(target - start) - Math.abs(cur - start);

            if (totalMinusTravled < pinPointOdo.getTicks(inchesForMinPower)) {
                power = minimumPowerSideways;
            } else if (totalMinusTravled < pinPointOdo.getTicks(inchesForTwiceMinPower)) {
                power = minimumPowerSideways*2;
            }

            //set power from movementcurve
            powerSidewaysDriveMotors(power*Math.signum(inches), RobotStates.yaw);

        }
        //stop when done
        stopDriveMotors();
    }


//    default void movementCustomArc(double inchesForward, double inchesLeft, Equation leftEqu, int straightFacing){
//        int flipForward = (int) Math.signum(inchesForward);
//        int flipLeft = (int) Math.signum(inchesLeft);
//        double y = leftEqu.apply(0.2) *  flipLeft;
//
//    }


    default void face(int degree){
        runtime.reset();
        double start = imu.getYaw();
        while(imu.notFacingTimer(degree, start, runtime, RobotConstants.defaultTimeWaitForTurn)){
            powerRotateDriveMotors(degree);
        }
        stopDriveMotors();
        RobotStates.yaw = degree;
    }
}
