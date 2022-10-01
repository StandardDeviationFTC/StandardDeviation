package org.firstinspires.ftc.teamcode.lib.misc;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

public class Util {

    public static VectorF getDriveStrafeVector(float xDrive, float yDrive, float robotAngle) {
        return Util.rotate(new VectorF(xDrive, yDrive), robotAngle);
    }

    public static VectorF rotate(VectorF vec, float angle) {
        float sin = (float) Math.sin(Math.toRadians(angle));
        float cos = (float) Math.cos(Math.toRadians(angle));

        float x1 = vec.get(0);
        float y1 = vec.get(1);

        float x2 = cos * x1 - sin * y1;
        float y2 = sin * x1 + cos * y1;

        return new VectorF(x2, y2);
    }

}
