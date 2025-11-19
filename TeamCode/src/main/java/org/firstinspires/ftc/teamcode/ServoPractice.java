package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@Disabled
@TeleOp(name = "Servo Practice", group = "Robot")
public class ServoPractice extends OpMode {

    Servo leftClaw;
    Servo rightClaw;
    Servo DUNKER;;

    @Override
    public void init() {
        leftClaw = hardwareMap.get(Servo.class, "Left Clawimen");
        rightClaw = hardwareMap.get(Servo.class, "Right Specclaw");
        DUNKER = hardwareMap.get(Servo.class, "DUNKER");
        int lift_height = 0;

    }

    @Override
    public void loop() {
        if (gamepad2.x) {
            leftClaw.setPosition(0.8);
            rightClaw.setPosition(0.2);
        } else {
            leftClaw.setPosition(1);
            rightClaw.setPosition(0);

        }

    }
    void DUNKERloop() {
        if (gamepad2.b) {
            DUNKER.setPosition(.23);
        }else {
            DUNKER.setPosition(0);
        }
    }
}
