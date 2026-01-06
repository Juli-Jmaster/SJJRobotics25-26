package org.firstinspires.ftc.teamcode.library.servo;

// a fixed set position servo class
public class FixedPositionServo extends Servob {
    private final double[] positions;

    public FixedPositionServo(String servoName, double[] positions){
        super(servoName);
        this.positions = positions;
    }

    public <E extends Enum<E>> void set(E enumValue){
        setPos(positions[enumValue.ordinal()]);
    }

    // TODO: DO WE NEED THIS ?
    //public double get(int num){
    //   return positions[num];
    //}
}
