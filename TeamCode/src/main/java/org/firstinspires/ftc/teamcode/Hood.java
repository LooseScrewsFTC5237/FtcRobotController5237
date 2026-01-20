package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
@Disabled
public class Hood {
    private Servo servoPos, ready, artifactIndicator;


    public void init(HardwareMap hwMap) {
        servoPos = hwMap.get(Servo.class, "hood");
        ready = hwMap.get(Servo.class, "ready");
        artifactIndicator = hwMap.get(Servo.class,"Artifact Indicator");
    }

    public void setServoPos(double angle) {
        servoPos.setPosition(angle);
    }

    public void setReadyPos(double position) {
        ready.setPosition(position);
    }

    public void setArtifactIndicatorPos(double position) {
        artifactIndicator.setPosition(position);
    }
}
