package org.firstinspires.ftc.teamcode.lib.misc;

public class Stopwatch {

    private long timerStartMillis;

    public Stopwatch() {
        restart();
    }

    public void restart() {
        this.timerStartMillis = System.currentTimeMillis();
    }

    public float getSeconds() {
        return getMillis() / 1000.0f;
    }

    public long getMillis() {
        return System.currentTimeMillis() - timerStartMillis;
    }

}
