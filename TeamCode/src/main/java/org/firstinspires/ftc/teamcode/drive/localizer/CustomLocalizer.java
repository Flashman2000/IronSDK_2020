package org.firstinspires.ftc.teamcode.drive.localizer;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.openftc.revextensions2.ExpansionHubMotor;

public class CustomLocalizer{

    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 0.7480314961;
    public static double GEAR_RATIO = 1;

    public static double LATERAL_DISTANCE = 7;
    public static double FORWARD_OFFSET = -10.9;

    public ExpansionHubMotor leftEncoder, rightEncoder, backEncoder;

    public CustomLocalizer(HardwareMap hardwareMap){
        leftEncoder = hardwareMap.get(ExpansionHubMotor.class, "rc");
        rightEncoder = hardwareMap.get(ExpansionHubMotor.class, "spool");
        backEncoder = hardwareMap.get(ExpansionHubMotor.class, "spool2");

        leftEncoder.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public double encoderTicksToInches(int ticks){
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

}
