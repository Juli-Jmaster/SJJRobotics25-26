package org.firstinspires.ftc.teamcode.library.servo;

public class ClawServo extends Servob {
    private double openPos;
    private double closePos;

    public ClawServo(String servoName, double openPos, double closePos) {
        super(servoName);
        this.closePos = closePos;
        this.openPos = openPos;
    }

    public void open(){
        setPos(openPos);
    }

    public void close(){
        setPos(closePos);
    }
}
