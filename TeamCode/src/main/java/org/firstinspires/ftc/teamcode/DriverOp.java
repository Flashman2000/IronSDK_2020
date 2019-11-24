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

        boolean heartbeat = false;
        boolean strobe = false;

        ElapsedTime time = new ElapsedTime();

        robot.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE);

        telemetry.addLine("Ready");

        waitForStart();

        time.reset();
        time.startTimeNanoseconds();
        telemetry.clearAll();

        while(opModeIsActive()){

            robot.teleActivity(gamepad1, gamepad2);

            //telemetry.addData("Time", time.time());
            //telemetry.update();

            if(time.time() >= 100 && !heartbeat && !strobe){
                robot.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED);
                heartbeat = true;
                strobe = true;
            }
            if(time.time() >= 110 && heartbeat && strobe){
                robot.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_RED);
            }
            if(time.time() >= 120 ){
                robot.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE);
                strobe = false;
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

