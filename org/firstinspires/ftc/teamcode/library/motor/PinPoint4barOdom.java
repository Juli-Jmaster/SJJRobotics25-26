package org.firstinspires.ftc.teamcode.library.motor;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.TypeConversion;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import java.nio.ByteOrder;
import java.util.Arrays;

public class PinPoint4barOdom {

    private final String name;
    public OdometryMotor odometryMotor;
    public GoBildaPinpointDriver pinPointDrive;
    public PinPoint4barOdom(String name) {
        odometryMotor= new OdometryMotor(name, OdometryMotor.WHEELTYPE.MM, 32, OdometryMotor.TYPE.TICKPERREV, 2000);
        this.name = name;
    }


    public void set(HardwareMap map) {
        pinPointDrive = map.get(GoBildaPinpointDriver.class, name);
    }
    public void update() {
        pinPointDrive.update();
    }

    //pass in the ticks and it returns the amount of inches those ticks are
    private double getInches(int ticks){
        return odometryMotor.getTicks(ticks);
    }

    //pass in inches you want and get the anount of tick to move those inches
    public int getTicks(double inches){
        return odometryMotor.getTicks(inches);
    }
    public int getTicksWithUpdate(double inches){
        pinPointDrive.update();
        return odometryMotor.getTicks(inches);
    }


}
