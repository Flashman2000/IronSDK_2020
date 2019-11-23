package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.configs.Monmon;

@TeleOp(name = "Teleop")
public class DriverOp extends LinearOpMode {

    Monmon robot = new Monmon();

    @Override
    public void runOpMode(){

        robot.initTele(hardwareMap);

        boolean blue = false;
        boolean red = false;

        ElapsedTime time = new ElapsedTime();
        String alliance = "";

        telemetry.addLine("Side? Start for Blue. Back for Red");
        telemetry.update();

        while(!blue && !red) {
            if (gamepad1.start) {
                blue = true;
            }
            if (gamepad1.back) {
                red = true;
            }
        }
        telemetry.clearAll();

        if(blue){
            robot.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_OCEAN_PALETTE);
            alliance = "Blue";
        }
        if(red){
            robot.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_LAVA_PALETTE);
            alliance = "Red";
        }

        telemetry.addData("Ready to go. Selected alliance", alliance);
        telemetry.update();

        waitForStart();

        time.reset();
        time.startTimeNanoseconds();
        telemetry.clearAll();

        while(opModeIsActive()){

            robot.teleActivity(gamepad1, gamepad2);

            //telemetry.addData("Time", time.time());
            //telemetry.update();

            if(time.time() >= 100 && blue){
                robot.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_BLUE);
            }
            if(time.time() >= 100 & red){
                robot.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED);
            }
            if(time.time() >= 120){
                robot.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE);
            }

            if(gamepad2.dpad_left){
                robot.turner.setPosition(0);
            }

            if(gamepad2.dpad_right){
                robot.turner.setPosition(1);
            }

            if(gamepad2.left_bumper){
                robot.i = robot.turner.getPosition();
                while(robot.i > 0 && opModeIsActive()){
                    robot.i = robot.i - 0.05;
                    robot.turner.setPosition(robot.i);
                    robot.teleActivity(gamepad1, gamepad2);
                    sleep(100);
                }
            }

            if (gamepad2.right_bumper){
                robot.i = robot.turner.getPosition();
                while(robot.i < 1 && opModeIsActive()) {
                    robot.i = robot.i + 0.05;
                    robot.turner.setPosition(robot.i);
                    robot.teleActivity(gamepad1, gamepad2);
                    sleep(100);
                }
            }


        }

    }

}

