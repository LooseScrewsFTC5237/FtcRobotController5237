package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DecodeRobot {

    private DcMotor frMotor;
    private DcMotor flMotor;
    private DcMotor brMotor;
    private DcMotor blMotor;
    private DcMotor singleLauncher;
    private double ticksPerRev; // revolution

    public void  init(HardwareMap hwMap) {
        //DC motor
        frMotor = hwMap.get(DcMotor.class, "front_right_motor");
        frMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        flMotor = hwMap.get(DcMotor.class, "front_left_motor");
        flMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        brMotor = hwMap.get(DcMotor.class, "back_right_motor");
        brMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        blMotor = hwMap.get(DcMotor.class, "back_left_motor");
        blMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        singleLauncher = hwMap.get(DcMotor.class, "single_launcher");
        singleLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ticksPerRev = singleLauncher.getMotorType().getTicksPerRev();

        singleLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        singleLauncher.setDirection(DcMotorSimple.Direction.REVERSE);
        //singleLauncher.setDirection(DcMotorSimple.Direction.FORWARD);



    }

    public void setMotorSpeed(double speed) {
         // accepts values from -1.0 -> 1.0
        frMotor.setPower(speed);
        brMotor.setPower(speed);
        flMotor.setPower(speed);
        blMotor.setPower(speed);
        singleLauncher.setPower(speed);
    }

    public double getMotorRevs() {
        return singleLauncher.getCurrentPosition() / ticksPerRev; // normalizing ticks to revolutions

    }
    public void setSingleLauncherZeroBehavior(DcMotor.ZeroPowerBehavior zeroBehavior) {
        singleLauncher.setZeroPowerBehavior(zeroBehavior);
    }
}
