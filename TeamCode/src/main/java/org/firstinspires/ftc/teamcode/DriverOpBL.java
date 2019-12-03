package org.firstinspires.ftc.teamcode;


import com.acmerobotics.roadrunner.drive.Drive;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.configs.Monmon;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREV;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;
import org.firstinspires.ftc.teamcode.util.MecanumPowers;
import org.firstinspires.ftc.teamcode.util.MecanumUtil;

@TeleOp(name = "Teleop Facing Right")
public class DriverOpBL extends LinearOpMode {

    SampleMecanumDriveREVOptimized robot;

    boolean headingAdjust = true;

    @Override
    public void runOpMode(){

        robot = new SampleMecanumDriveREVOptimized(hardwareMap);

        boolean heartbeat = false;
        boolean strobe = false;

        ElapsedTime time = new ElapsedTime();

        robot.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE);

        robot.primeServo();
        telemetry.addLine("Ready");


        waitForStart();

        time.reset();
        time.startTimeNanoseconds();
        telemetry.clearAll();

        while(opModeIsActive()){

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



            if(headingAdjust) {
                double leftX = -gamepad1.left_stick_y; //-gamepad1.left_stick_x;
                double leftY = gamepad1.left_stick_x;
                double angle = -Math.atan2(leftY, leftX) + Math.PI / 2;
                angle -= robot.getExternalHeading();
                double driveScale = Math.sqrt(Math.pow(leftX, 2) + Math.pow(leftY, 2));
                driveScale = Range.clip(driveScale, 0, 1);

                double turn = Math.copySign(
                        Math.pow(-gamepad1.right_stick_x, 2),
                        -gamepad1.right_stick_x);

                MecanumPowers powers = MecanumUtil.powersFromAngle(angle, driveScale, turn);
                robot.setPowers(powers);
            }
            if(!headingAdjust){
                double leftX = -gamepad1.left_stick_x; //-gamepad1.left_stick_x;
                double leftY = -gamepad1.left_stick_y;
                double angle = -Math.atan2(leftY, leftX) + Math.PI / 2;
                double driveScale = Math.sqrt(Math.pow(leftX, 2) + Math.pow(leftY, 2));
                driveScale = Range.clip(driveScale, 0, 1);

                double turn = Math.copySign(
                        Math.pow(-gamepad1.right_stick_x, 2),
                        -gamepad1.right_stick_x);

                MecanumPowers powers = MecanumUtil.powersFromAngle(angle, driveScale, turn);
                robot.setPowers(powers);
            }

            if(gamepad1.start){
                headingAdjust = false;
            }
            if(gamepad1.back){
                headingAdjust = true;
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

