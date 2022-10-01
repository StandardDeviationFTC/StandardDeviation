package org.firstinspires.ftc.teamcode.lib.input;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.HashMap;
import java.util.Map;

public class Controls {

    private static Gamepad gamepad;
    private static Map<String, Input> bindings = new HashMap<String, Input>();

    public static void setGamepad(Gamepad pad) {
        gamepad = pad;
    }

    public static void bindInput(String name, Input binding) {
        bindings.put(name, binding);
    }

    public static boolean getBoolean(String name) {
        float rawInput = getRawInput(name);
        return (rawInput == 1);
    }

    public static float getFloat(String name) {
        return getRawInput(name);
    }

    private static float getRawInput(String name) {
        Input input = bindings.get(name);
        return input.getValue(gamepad);
    }

}
