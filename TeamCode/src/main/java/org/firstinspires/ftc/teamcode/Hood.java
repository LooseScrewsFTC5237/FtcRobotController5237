package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Hood {
    private Servo servoPos, ready;


    public void init(HardwareMap hwMap) {
        servoPos = hwMap.get(Servo.class, "hood");
        ready = hwMap.get(Servo.class, "ready");
    }

    public void setServoPos(double angle) {
        servoPos.setPosition(angle);
    }

    public void setReadyPos(double position) {
        ready.setPosition(position);
    }

}
