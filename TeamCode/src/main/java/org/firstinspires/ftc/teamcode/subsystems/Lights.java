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


    public Lights(HardwareMap hwMap) {
        leftLockedOnLight = hwMap.get(Servo.class,"LeftLockedOnLight");
        rightLockedOnLight = hwMap.get(Servo.class,"RightLockedOnLight");
//        LightStrip = hwMap.get(RevBlinkinLedDriver.class, "test light strip");
    }

    public void
    turnLeftLockedOnLightOn(double color) {
        leftLockedOnLight.setPosition(color);
    }

    public void
    turnRightLockedOnLightOn(double color) {
        rightLockedOnLight.setPosition(color);
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
