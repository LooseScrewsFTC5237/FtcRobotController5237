package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Hood {
    private Servo servoPos;


    public void init(HardwareMap hwMap) {
     servoPos = hwMap.get(Servo.class, "hood");
    }

    public void setServoPos(double angle) {
        servoPos.setPosition(angle);
    }


}
