package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.configs.Monmon;

@Disabled
public class Gain_Test extends LinearOpMode {

    Monmon robot = new Monmon();

    @Override
    public void runOpMode(){
        robot.initAuto(hardwareMap, telemetry);

        waitForStart();

        robot.strafeLeftWithEnc(0.6, 5000, this);
        robot.resetEncoders();
        robot.strafeRightWithEnc(0.6, 5000, this);
    }

}
