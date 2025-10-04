package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class DcMotorPractice extends OpMode {
    DecodeRobot bench = new DecodeRobot();

    @Override
    public void init() {
        bench.init(hardwareMap);
    }


    @Override
    public  void loop() {
        bench.setMotorSpeed(0.5);
    }
}
