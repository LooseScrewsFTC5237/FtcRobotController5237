package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
@Disabled
public class distancesensor {
    private DistanceSensor distance;

    public void init(HardwareMap hwMap) {
        distance = hwMap.get(DistanceSensor.class, "distance_sensor");
    }

    public double getDistance() {
       return distance.getDistance(DistanceUnit.CM);
    }
}
