package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.configs.Monmon;

@Autonomous(name="Turn Test")
public class turnTest extends LinearOpMode {

    Monmon robot = new Monmon();
    
    @Override
    public void runOpMode(){

        robot.initAuto(hardwareMap, telemetry, this, false);

        robot.turnLeftGyro(90, this);

        sleep(2500);

        robot.turnLeft180();

    }

}
