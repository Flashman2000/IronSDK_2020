package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.configs.Monmon;

@Autonomous(name="Gain Test")
public class Gain_Test extends LinearOpMode {

    Monmon robot = new Monmon();

    @Override
    public void runOpMode(){
        robot.initAuto(hardwareMap, telemetry);

        waitForStart();

        robot.fwdWithEncoder(0.6, 2000, this);
    }

}
