package org.firstinspires.ftc.teamcode;

import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.BLACK;
import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.GREEN;
import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.RED;
import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.SINELON_FOREST_PALETTE;
import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.SINELON_OCEAN_PALETTE;
import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.YELLOW;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
@Disabled
public class Hood {
    private Servo servoPos, ready, artifactIndicator/*, LEDs*/;
    private RevBlinkinLedDriver LEDs;


    public void init(HardwareMap hwMap) {
        servoPos = hwMap.get(Servo.class, "hood");
        ready = hwMap.get(Servo.class, "ready");
        artifactIndicator = hwMap.get(Servo.class,"Artifact Indicator");
        LEDs = hwMap.get(RevBlinkinLedDriver.class, "LEDs");
    }


    public void setServoPos(double angle) {
        servoPos.setPosition(angle);
    }

    public void setReadyPos(double position) {
        ready.setPosition(position);
    }
    public void setReadyLEDPos(int color) {
        if (color == 1) {
            LEDs.setPattern(GREEN);
        } else if (color == 2){
            LEDs.setPattern(YELLOW);
        }  else {
            LEDs.setPattern(RED);
        }
    }
    public void setArtifactIndicatorPos(double position) {
        artifactIndicator.setPosition(position);
    }
}
