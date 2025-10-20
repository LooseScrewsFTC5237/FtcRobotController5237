package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class hoodtest extends OpMode {
    Hood hood = new Hood();

    @Override
    public void init() {
        hood.init(hardwareMap);
        hood.setServoPos(0.5);
    }

    @Override
    public void loop() {

    }
}

