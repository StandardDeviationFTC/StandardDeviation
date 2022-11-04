package org.firstinspires.ftc.teamcode.lib.input;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.function.Function;

public enum Input {
    SQUARE(pad -> toFloat(pad.square)), TRIANGLE(pad -> toFloat(pad.triangle)),
    CIRCLE(pad -> toFloat(pad.circle)), X(pad -> toFloat(pad.cross)),
    LT(pad -> pad.left_trigger), RT(pad -> pad.right_trigger),
    LB(pad -> toFloat(pad.left_bumper)), RB(pad -> toFloat(pad.right_bumper)),
    DPAD_LEFT(pad -> toFloat(pad.dpad_left)), DPAD_RIGHT(pad -> toFloat(pad.dpad_right)),
    DPAD_UP(pad -> toFloat(pad.dpad_up)), DPAD_DOWN(pad -> toFloat(pad.dpad_down)),
    LEFT_STICK_X(pad -> pad.left_stick_x), LEFT_STICK_Y(pad -> pad.left_stick_y),
    RIGHT_STICK_X(pad -> pad.right_stick_x), RIGHT_STICK_Y(pad -> pad.right_stick_y);

    private static float toFloat(boolean b) {
        return b ? 1.0f : 0.0f;
    }
    
    private Function<Gamepad, Float> valueFunction;

    private Input(Function<Gamepad, Float> valueFunction) {
        this.valueFunction = valueFunction;
    }

    public float getValue(Gamepad gamepad) {
        return valueFunction.apply(gamepad);
    }
}
