package org.firstinspires.ftc.teamcode.subsystems;

import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.BLUE;
import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_FOREST_PALETTE;
import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_LAVA_PALETTE;
import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_OCEAN_PALETTE;
import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_PARTY_PALETTE;
import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.GREEN;
import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.RED;
import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.YELLOW;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
public class Lights {
    private Servo leftLockedOnLight; // not a servo it's a light
    private Servo rightLockedOnLight; // also not a servo it's a light
//    private RevBlinkinLedDriver LightStrip;

    private double lightChange = 0;
    private boolean fadingUp = true;

    public Lights(HardwareMap hwMap) {
        leftLockedOnLight = hwMap.get(Servo.class,"LeftLockedOnLight");
        rightLockedOnLight = hwMap.get(Servo.class,"RightLockedOnLight");
//        LightStrip = hwMap.get(RevBlinkinLedDriver.class, "test light strip");
    }

    public void fadeLight(){
//        if (lightChange < 0.055){
//            if (fadingUp){
//                lightChange = lightChange + 0.001;
//            } else {
//                lightChange = lightChange - 0.001;
//                if (lightChange < 0){
//                    fadingUp = true;
//                    lightChange = lightChange = 1;
//                }
//            }
//        } else {
//            fadingUp = false;
//            lightChange = lightChange - 0.001;
//        }
    }

    public void turnLeftLockedOnLightOn(double color) {
        leftLockedOnLight.setPosition(color + lightChange);
    }

    public void turnRightLockedOnLightOn(double color) {
        rightLockedOnLight.setPosition(color + lightChange);
    }

//    public void turnLightStripOn(int color) {
//        if (color == 1) {
//            LightStrip.setPattern(COLOR_WAVES_OCEAN_PALETTE);
//        } else if (color == 2) {
//            LightStrip.setPattern(COLOR_WAVES_FOREST_PALETTE);
//        } else if (color == 3) {
//            LightStrip.setPattern(COLOR_WAVES_LAVA_PALETTE);
//        } else if (color == 4){
//            LightStrip.setPattern(COLOR_WAVES_PARTY_PALETTE);
//        }
//    }

}
