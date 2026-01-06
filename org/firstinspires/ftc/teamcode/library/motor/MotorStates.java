package org.firstinspires.ftc.teamcode.library.motor;

import org.firstinspires.ftc.teamcode.library.RobotConstants;

public class MotorStates extends Motor{
    private final int[] positions;

    public MotorStates(String name, boolean reversed, boolean brake, boolean encoders, int[] positions) {
        super(name, reversed, brake, encoders);
        this.positions = positions;
    }
    public <E extends Enum<E>> void set(E enumValue){
        move(positions[enumValue.ordinal()]);
    }

    @Override
    public void reset() {
        move(0, RobotConstants.defaultPower);
    }
}
