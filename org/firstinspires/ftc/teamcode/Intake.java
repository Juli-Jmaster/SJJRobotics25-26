package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class Intake extends LinearOpMode {

    private DcMotorEx m1;
    private DcMotorEx m2;

    @Override
    public void runOpMode() throws InterruptedException {
        m1 = hardwareMap.get(DcMotorEx.class, "m1");
        m1.setDirection(DcMotorSimple.Direction.REVERSE);
        m2 = hardwareMap.get(DcMotorEx.class, "m2");
        m2.setDirection(DcMotorSimple.Direction.REVERSE);



        waitForStart();

        while(opModeIsActive()){
            m1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            m2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            if (gamepad1.x){
//                m1.setPower(1);
                m2.setPower(1);

            }
            if (gamepad1.y){
                m2.setPower(.53);

            }
            if(gamepad1.a){
                m2.setPower(0);
                m2.setVelocity(10);
            }
            if (gamepad1.b){
                m1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                m2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                m1.setPower(0);
                m2.setPower(0);
            }
             telemetry.addData("f", m2.getVelocity());
             telemetry.addData("f", m2.getPower());
             telemetry.addData("f", m2.getZeroPowerBehavior());
//             telemetry.addData("d", m2.getVelocity(AngleUnit.DEGREES));
// //            telemetry.addData("d", m2.(AngleUnit.DEGREES));
// //            telemetry.addData("d", m2.(AngleUnit.DEGREES));

// //            telemetry.addData("d", m2.(AngleUnit.DEGREES));
//             telemetry.update();
        }
    }
}







