package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class PIDFTesting extends LinearOpMode {

    private DcMotorEx shoot;
    private int preset1 = 100;
    private int preset2 = 1000;
    private int velocity = 0;
    private boolean onceB1 = false;
    private boolean onceB2 = false;

    @Override
    public void runOpMode() throws InterruptedException {
        shoot = hardwareMap.get(DcMotorEx.class, "motor");
        shoot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shoot.setVelocityPIDFCoefficients(
                0.0,   // P
                0.0,   // I
                0.0,   // D
                0.0   // F
        );
        waitForStart();

        while(opModeIsActive()){
            if (gamepad1.x){
                velocity = preset1;

            }
            if (gamepad1.y){
                velocity =preset2;
            }
            if(gamepad1.a){
                shoot.setPower(0);
                shoot.setVelocity(0);
            }
            if (gamepad1.b){
              shoot.setPower(0.5);
            }
            if (gamepad1.start && !(onceB1)) {
                velocity += 1;
                onceB1 = true;
            } else {
                onceB1 = false;
            }

            if (gamepad1.back && !(onceB2)) {
                velocity -= 1;
                onceB2 = true;
            } else {
                onceB2 = false;
            }
            shoot.setVelocity(velocity);
            telemetry.addData("Velocity: ", shoot.getVelocity());
            telemetry.addData("Power:", shoot.getPower());
            telemetry.update();
        }
    }
}
