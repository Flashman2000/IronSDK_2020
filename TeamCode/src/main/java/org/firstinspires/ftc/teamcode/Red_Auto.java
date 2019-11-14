package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.configs.Monmon;

@Autonomous(name="Test Red")
public class Red_Auto extends LinearOpMode {

    Monmon robot = new Monmon();
    String SkystoneLocation;

    @Override
    public void runOpMode(){

        robot.initAuto(hardwareMap, telemetry);

        robot.rightArm.setPosition(0.4);

        waitForStart();

        robot.releaseBack();
        robot.releaseFront();

        telemetry.clearAll();

        SkystoneLocation = robot.detectSkystone();

        telemetry.addData("Position", SkystoneLocation);
        telemetry.update();

        if(SkystoneLocation == "Left" || SkystoneLocation == "NOT FOUND"){

            robot.diagRightWithEnc(0.75, 0.5, 1300, this);
            robot.resetEncoders();
            robot.strafeRightWithEnc(0.6, 800, this);
            robot.killAll();
            robot.resetEncoders();
            robot.grabFront();
            sleep(800);
            robot.strafeLeftWithEnc(0.6, 400, this);
            robot.resetEncoders();
            robot.bckWithEncoder(0.6, 3300, this);
            robot.selfCorrect();
            robot.killAll();
            robot.resetEncoders();
            robot.releaseFront();
            sleep(500);
            robot.fwdWithEncoder(0.6, 3900, this);
            robot.fwdWithTime(0.15, 5000, this);
            robot.resetEncoders();
            robot.strafeRightWithEnc(0.6, 300, this);
            robot.killAll();
            robot.resetEncoders();
            robot.grabFront();
            sleep(800);
            robot.strafeLeftWithEnc(0.6, 600, this);
            robot.selfCorrect();
            robot.resetEncoders();
            robot.bckWithEncoder(0.6, 1500, this);
            robot.bckWithEncoder(1, 4600, this);
            robot.killAll();
            robot.resetEncoders();
            robot.releaseFront();
            sleep(500);
            robot.fwdWithEncoder(0.6, 400, this);
            robot.killAll();
            robot.resetEncoders();

        }

        if(SkystoneLocation == "Center"){
            robot.diagRightWithEnc(0.75, 0.5, 1300, this);
            robot.resetEncoders();
            robot.strafeRightWithEnc(0.6, 800, this);
            robot.killAll();
            robot.resetEncoders();
            robot.grabBack();
            sleep(800);
            robot.strafeLeftWithEnc(0.6, 400, this);
            robot.resetEncoders();
            robot.bckWithEncoder(0.6, 3300, this);
            robot.selfCorrect();
            robot.killAll();
            robot.resetEncoders();
            robot.releaseBack();
            sleep(500);
            robot.fwdWithEncoder(0.6, 3900, this);
            robot.fwdWithTime(0.15, 5000, this);
            robot.resetEncoders();
            robot.strafeRightWithEnc(0.6, 300, this);
            robot.killAll();
            robot.resetEncoders();
            robot.grabBack();
            sleep(800);
            robot.strafeLeftWithEnc(0.6, 600, this);
            robot.selfCorrect();
            robot.resetEncoders();
            robot.bckWithEncoder(0.6, 1500, this);
            robot.bckWithEncoder(1, 4600, this);
            robot.killAll();
            robot.resetEncoders();
            robot.releaseBack();
            sleep(500);
            robot.fwdWithEncoder(0.6, 400, this);
            robot.killAll();
            robot.resetEncoders();

        }

        if(SkystoneLocation == "Right"){

            robot.diagLeftWithEnc(-0.6, -0.45, -1100, this);
            robot.resetEncoders();
            robot.strafeRightWithEnc(0.6, 550, this);
            robot.selfCorrect();
            robot.killAll();
            robot.resetEncoders();
            robot.grabBack();
            sleep(800);
            robot.strafeLeftWithEnc(0.6, 550, this);
            robot.resetEncoders();
            robot.bckWithEncoder(0.6, 3000, this);
            robot.selfCorrect();
            telemetry.addData("First heading", robot.imu.getAngularOrientation().firstAngle);
            telemetry.update();
            robot.killAll();
            robot.resetEncoders();
            robot.releaseBack();
            sleep(500);
            robot.fwdWithEncoder(0.6, 3800, this);
            robot.selfCorrect();
            telemetry.addData("Second heading", robot.imu.getAngularOrientation().firstAngle);
            telemetry.update();
            robot.resetEncoders();
            robot.strafeRightWithEnc(0.6, 300, this);
            robot.killAll();
            robot.resetEncoders();
            robot.grabFront();
            sleep(800);
            robot.strafeLeftWithEnc(0.6, 600, this);
            robot.selfCorrect();
            telemetry.addData("Third heading", robot.imu.getAngularOrientation().firstAngle);
            telemetry.update();
            robot.resetEncoders();
            robot.bckWithEncoder(0.6, 1000, this);
            robot.bckWithEncoder(1, 3500, this);
            robot.selfCorrect();
            telemetry.addData("Fourth heading", robot.imu.getAngularOrientation().firstAngle);
            telemetry.update();
            robot.killAll();
            robot.resetEncoders();
            robot.releaseFront();
            sleep(500);
            robot.fwdWithEncoder(0.6, 800, this);
            robot.killAll();
            robot.resetEncoders();


        }

    }

}
