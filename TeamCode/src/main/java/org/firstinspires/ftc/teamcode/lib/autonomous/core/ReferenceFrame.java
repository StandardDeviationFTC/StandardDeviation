package org.firstinspires.ftc.teamcode.lib.autonomous.core;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.lib.misc.Constants;
import org.firstinspires.ftc.teamcode.lib.misc.Util;

public enum ReferenceFrame {

    ABSOLUTE(new VectorF(0, 0, 0), 0),
    RED_LEFT(new VectorF(Constants.HALF_FIELD_INCHES - Constants.HALF_TILE_INCHES, -Constants.ONE_AND_HALF_TILE_INCHES, 0), 0),
    RED_RIGHT(new VectorF(Constants.HALF_FIELD_INCHES - Constants.HALF_TILE_INCHES, Constants.ONE_AND_HALF_TILE_INCHES, 0), 0);

    VectorF origin;
    float rotation;

    private ReferenceFrame(VectorF origin, float rotation) {
        this.origin = origin;
        this.rotation = rotation;
    }

    public VectorF getRelativePosition(VectorF absolutePosition) {
        absolutePosition.subtract(origin);
        return Util.rotate(absolutePosition, rotation);
    }

}
