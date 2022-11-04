package org.firstinspires.ftc.teamcode.lib.input;

public class ToggleButton {

    private String inputName;
    private boolean state;
    private boolean isPressed;

    public ToggleButton(String inputName) {
        this.inputName = inputName;
    }

    public ToggleButton(String inputName, boolean state) {
        this.inputName = inputName;
        this.state = state;
    }

    public boolean update() {
        if(!isPressed && Controls.getBoolean(inputName)) {
            isPressed = true;
            state = !state;

            return true;
        } else if(!Controls.getBoolean(inputName)) {
            isPressed = false;
        }

        return false;
    }

    public boolean getState() {
        return this.state;
    }

}
