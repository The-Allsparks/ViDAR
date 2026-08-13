package org.firstinspires.ftc.teamcode.vidar.fusion;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.junit.jupiter.api.Test;

import java.util.concurrent.atomic.AtomicReference;

import static org.junit.jupiter.api.Assertions.*;

class FieldPoseContextTest {

    @Test
    void setOdomSupplierRebindsAfterBootstrapStyleReuse() {
        AtomicReference<Pose2D> autoOdom =
                new AtomicReference<>(new Pose2D(DistanceUnit.INCH, 1, 0, AngleUnit.DEGREES, 0));
        AtomicReference<Pose2D> teleOdom =
                new AtomicReference<>(new Pose2D(DistanceUnit.INCH, 9, 0, AngleUnit.DEGREES, 0));

        FieldPoseContext ctx = new FieldPoseContext(autoOdom::get);
        assertEquals(1.0, ctx.odomSupplier().get().getX(DistanceUnit.INCH), 1e-9);

        ctx.setOdomSupplier(teleOdom::get);
        assertEquals(9.0, ctx.odomSupplier().get().getX(DistanceUnit.INCH), 1e-9);
    }
}
