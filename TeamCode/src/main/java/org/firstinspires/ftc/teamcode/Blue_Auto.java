package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.configs.Monmon;

@Autonomous(name = "Test Blue")
public class Blue_Auto extends LinearOpMode {

    Monmon robot = new Monmon();
    String SkystoneLocation;

    @Override
    public void runOpMode(){

        robot.initAuto(hardwareMap, telemetry, this, true);

        robot.releaseBack();
        robot.releaseFront();

        telemetry.clearAll();

        SkystoneLocation = robot.detectSkystone();

        telemetry.addData("Position", SkystoneLocation);
        telemetry.update();

        robot.alignSkystone(SkystoneLocation);
        robot.killAll();
        robot.resetEncoders();

        robot.closePipeline(); //avoid RAM leakage

        if(SkystoneLocation == "Left" || SkystoneLocation == "NOT FOUND"){

            robot.strafeRightWithEnc(0.6, 1832, this);
            robot.killAll();

            robot.resetEncoders();
            robot.bckWithEncoder(0.6, 80, this);
            robot.killAll();
            robot.resetEncoders();
            robot.grabFront();
            sleep(800);
            robot.strafeLeftWithEnc(0.6, 600, this);
            robot.resetEncoders();
            robot.fwdWithEncoder(0.85, 3000, this);
            robot.killAll();
            robot.resetEncoders();
            robot.releaseFront();
            sleep(500);
            robot.bckWithEncoder(0.6, 3800, this);
            robot.resetEncoders();
            robot.strafeRightWithEnc(0.6, 400, this);
            robot.killAll();
            robot.resetEncoders();
            robot.grabBack();
            sleep(800);
            robot.strafeLeftWithEnc(0.6, 750, this);
            robot.resetEncoders();
            robot.fwdWithEncoder(1, 3800, this);
            robot.killAll();
            robot.resetEncoders();
            robot.releaseBack();
            sleep(500);
            robot.bckWithEncoder(1, 800, this);
            robot.killAll();
            robot.resetEncoders();
            robot.leftArm.setPosition(1);
            robot.rightArm.setPosition(0);
            sleep(2000);
            robot.strafeRightWithTime(0.3, 2000, this);
            robot.killAll();
            robot.resetEncoders();

        }

        if(SkystoneLocation == "Center"){

            robot.strafeRightWithEnc(0.6, 1832, this);
            robot.killAll();
            robot.resetEncoders();
            robot.fwdWithEncoder(0.6, 80, this);
            robot.killAll();
            robot.resetEncoders();
            robot.grabFront();
            sleep(800);
            robot.strafeLeftWithEnc(0.6, 550, this);
            robot.resetEncoders();
            robot.fwdWithEncoder(0.85, 3300, this);
            robot.killAll();
            robot.resetEncoders();
            robot.releaseFront();
            sleep(500);
            robot.bckWithEncoder(0.6, 4200, this);
            robot.resetEncoders();
            robot.killAll();
            robot.resetEncoders();
            robot.strafeRightWithEnc(0.6, 550, this);
            robot.killAll();
            robot.resetEncoders();
            robot.grabBack();
            sleep(800);
            robot.strafeLeftWithEnc(0.6, 800, this);
            robot.resetEncoders();
            robot.fwdWithEncoder(1, 1500, this);
            robot.fwdWithEncoder(1, 4400, this);
            robot.killAll();
            robot.resetEncoders();
            robot.releaseBack();
            sleep(500);
            robot.bckWithEncoder(0.6, 800, this);
            robot.killAll();
            robot.resetEncoders();
            robot.leftArm.setPosition(1);
            robot.rightArm.setPosition(0);
            sleep(2000);
            robot.strafeRightWithTime(0.3, 2000, this);
            robot.killAll();
            robot.resetEncoders();

        }

        if(SkystoneLocation == "Right"){

            robot.strafeRightWithEnc(0.6, 1832, this);
            robot.killAll();
            robot.resetEncoders();
            robot.fwdWithEncoder(0.6, 120, this);
            robot.killAll();
            robot.resetEncoders();
            robot.grabBack();
            sleep(800);
            robot.strafeLeftWithEnc(0.6, 550, this);
            robot.resetEncoders();
            robot.fwdWithEncoder(0.85, 3300, this);
            robot.killAll();
            robot.resetEncoders();
            robot.releaseBack();
            sleep(500);
            robot.bckWithEncoder(0.85, 3500, this);
            robot.resetEncoders();
            robot.bckWithTime(0.3,1500, this);
            robot.killAll();
            robot.resetEncoders();
            robot.strafeRightWithEnc(0.6, 650, this);
            robot.killAll();
            robot.resetEncoders();
            robot.grabBack();
            sleep(800);
            robot.strafeLeftWithEnc(0.6, 800, this);
            robot.resetEncoders();
            robot.fwdWithEncoder(1, 1500, this);
            robot.fwdWithEncoder(1, 4400, this);
            robot.killAll();
            robot.resetEncoders();
            robot.releaseBack();
            sleep(500);
            robot.bckWithEncoder(0.6, 400, this);
            robot.killAll();
            robot.resetEncoders();
            robot.leftArm.setPosition(1);
            robot.rightArm.setPosition(0);
            sleep(2000);
            robot.strafeRightWithTime(0.3, 2000, this);
            robot.killAll();
            robot.resetEncoders();


        }

    }
}
