/*
Copyright 2026 FIRST Tech Challenge Team 6298

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class hood extends LinearOpMode {


    @Configureable

    private DcMotorEx shoot;
    private Servo hood;

    @Override
    public void runOpMode() throws InterruptedException {
        shoot = hardwareMap.get(DcMotorEx.class, "shoot");
        shoot.setDirection(DcMotorSimple.Direction.FORWARD);
        hood = hardwareMap.get(Servo.class, "hood");
        shoot.setVelocityPIDFCoefficients(
                0.0,   // P
                0.0,   // I
                0.0,   // D
                0.0   // F
        );


        waitForStart();

        while(opModeIsActive()){
            shoot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            if (gamepad1.x){
//                m1.setPower(1);
                shoot.setVelocity(1000);
                hood.setPosition(0);

            }
            if (gamepad1.y){
                //m2.setPower(.53);

            }
            if(gamepad1.a){
                shoot.setPower(0);
                //shoot.setVelocity(-200);
            }
            if (gamepad1.b){
                // m1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                // m2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                // m1.setPower(0);
                //m2.setPower(0);
            }
            telemetry.addData("f", shoot.getVelocity());
            //  telemetry.addData("f", shoot.getPower());
            //telemetry.addData("f", shoot.ZeroPowerBehavior());
//             telemetry.addData("d", m2.getVelocity(AngleUnit.DEGREES));
// //            telemetry.addData("d", m2.(AngleUnit.DEGREES));
// //            telemetry.addData("d", m2.(AngleUnit.DEGREES));

// //            telemetry.addData("d", m2.(AngleUnit.DEGREES));
            telemetry.update();
        }
    }
}
