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
        waitForStart();
        robot.resetEncoders();
        telemetry.addLine("Pass 1");
        telemetry.update();
        robot.turnLeftGyro(80, this);
        telemetry.addLine("Pass 2");
        telemetry.update();
        robot.setMaintainedHeading(90);
        robot.killAll();
        robot.resetEncoders();
        robot.strafeRightWithEnc(0.6, 4000, this);
        robot.resetEncoders();
        telemetry.addLine("Pass 3");
        telemetry.update();
        robot.strafeLeftWithEnc(0.6, 4000, this);
        robot.killAll();
        robot.resetEncoders();
        telemetry.addLine("Pass 4");
        telemetry.update();
        sleep(5000);
        robot.turnLeft180(this);
        robot.killAll();
        sleep(2500);

    }

}
