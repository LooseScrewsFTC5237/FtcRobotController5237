package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp
public class DistanceTest extends OpMode {
    distancesensor bench = new distancesensor();
    @Override
    public void init() {
        bench.init(hardwareMap);
    }
    @Override
    public void loop() {
        telemetry.addData("Distance", bench.getDistance());
    }
}
