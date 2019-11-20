package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.configs.Monmon;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Teleop")
public class DriverOp extends LinearOpMode {

    Monmon robot = new Monmon();

    @Override
    public void runOpMode(){

        robot.initTele(hardwareMap);

        waitForStart();

        while(opModeIsActive()){

            robot.teleActivity(gamepad1, gamepad2);

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
