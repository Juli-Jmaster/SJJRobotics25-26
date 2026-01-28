package org.firstinspires.ftc.teamcode.library;

import org.firstinspires.ftc.teamcode.library.motor.OdometryMotor;

public interface RobotConstants {

    //* Overall Default Max power
    double defaultPower = 0.8;

    //* MovementCurves.java
    //THRESHOLD should be equal to the minimum amount or little higher of power it takes to move the robot forward or directly sideways
    double minimumPowerToMove = 0.15;  // or .2

    //* Drive MovementCurves and settings
    // all drive motor set to brake for Zero Power Behavior
    boolean driveMotorBrake  = true;
    //if all motor have encoders cables attached and want to use those instead
    boolean drivemotorEncoders = false;

    int defaultDriveMovementCurve = MovementCurves.ROUNDEDSQUARE; //TODO: find correct one
    double defaultDriveMultipier = 1;

    //* Sideways MovementCurves
    //? DO WE NEED
    int defaultTurnMovementCurve = MovementCurves.ROUNDEDSQUARE; //TODO: find correct one
    double defaultTurnMultipier = 1;


    //* IMU MovementCurves
    int defaultIMUTurnMovementCurve = MovementCurves.PARAMETRIC;
    double defaultIMUTurnMultipier = .6;
    double minimumPowerToTurn = 0.1;
    // the default tolerance for when turning
    //how many degrees off it can be before stopping
    // Ex. is 3 degreees of from target say robot got to the position
    int defaultToleranceFace = 3;
    //time before the notFacing() return false as it may be stuck in being exact
    int defaultTimeWaitForTurn = 3;

    //* Default Odometry settings
    //also found in class OdemetryMotor.java
    //the wheel type is the type of units used for the diameter of the wheel
    //  Ex. if the diamemter of the wheel is 48mm then it is WHEELTYPE.MM
    //this is the type of data the is found on the GoBuilda website
    //  which is called Encoder Resolution on the specs of motor or odemetry pod
    //  it either be  "Parts Per Revolution (PPR)" or "Countable Events per Revolution"
    //  "PPR" is TYPE.PPR and "Countable Events per Revolution" is TYPE.TICKPERREV
    OdometryMotor.WHEELTYPE diameterLengthType = OdometryMotor.WHEELTYPE.MM;
    int diameterLength = 48;

    OdometryMotor.TYPE ticksPerType = OdometryMotor.TYPE.TICKPERREV;
    int ticksPerTypeNumber = 2000;


    String frontRightMotorName = "frontright" ;
    boolean frontRightReversed = false;

    String frontLeftMotorName = "frontleft";
    boolean frontLeftReversed = false;

    String backRightMotorName = "backright";
    boolean backRightReversed = false;

    String backLeftMotorName = "backleft";
    boolean backLeftReversed = false ;
    //? old (DO NEED?)
    // do not modify;used for when running
    int itorator = 0;
    int straight = 0;
    int sideways = 1;

    //used for while in drive loop
    //only use if statements so drive can still work effectively
    default void whileDrive(int MOVE){

    }
}
