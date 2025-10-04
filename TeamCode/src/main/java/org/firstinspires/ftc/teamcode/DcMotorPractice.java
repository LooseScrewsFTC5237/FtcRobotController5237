package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class DcMotorPractice extends OpMode {
    DecodeRobot bench = new DecodeRobot();

    @Override
    public void init() {
        bench.init(hardwareMap);
    }


    @Override
    public  void loop() {
        double motorSpeed = gamepad1.left_stick_y;

        bench.setMotorSpeed(motorSpeed);

        if (gamepad1.b) {
            bench.setSingleLauncherZeroBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        else if (gamepad1.a) {
         bench.setSingleLauncherZeroBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        telemetry.addData("Motor Revs", bench.getMotorRevs());
    }
}


