package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.configs.Monmon;

@Autonomous(name = "Test Blue Plate")
public class Blue_With_Plate extends LinearOpMode {

    Monmon robot = new Monmon();
    String SkystoneLocation;

    @Override
    public void runOpMode(){

        robot.initAuto(hardwareMap, telemetry, this, true);

        waitForStart();

        robot.releaseBack();
        robot.releaseFront();

        telemetry.clearAll();

        SkystoneLocation = robot.detectSkystone();

        telemetry.addData("Position", SkystoneLocation);
        telemetry.update();

        robot.alignSkystone(SkystoneLocation);
        robot.killAll();
        robot.resetEncoders();

        robot.closePipeline(); //avoid RAM leakaation == "NOT FOUNDge
                                                                 
        if(SkystoneLocation == "Left" || SkystoneLocation == "NOT FOUND"){
            robot.strafeRightWithEnc(0.6, 1832, this);
            robot.killAll();
            robot.resetEncoders();
            robot.bckWithEncoder(0.6, 80, this);
            robot.killAll();
            robot.resetEncoders();
            robot.grabFront();
            sleep(800);
            robot.strafeLeftWithEnc(0.6, 350, this);
            robot.killAll();
            robot.resetEncoders();
            robot.turnLeftGyro(80, this);
            robot.killAll();
            robot.setMaintainedHeading(85);
            robot.killAll();
            robot.resetEncoders();
            robot.strafeRightWithEnc(0.8, 4500, this);
            robot.releaseFront();
            robot.strafeRightWithEnc(0.68, 6000, this);
            //robot.strafeRightWithTime(0.2, 2000, this);
            robot.resetEncoders();
            //robot.setMaintainedHeading(90);
            //robot.strafeLeftWithEnc(0.3, 900, this);
            robot.killAll();
            robot.resetEncoders();
            //robot.strafeRightWithEnc(0.3, 800, this);
            robot.resetEncoders();
            robot.bckWithTime(0.3, 1000, this);
            robot.killAll();
            robot.resetEncoders();
            robot.clampBack();
            sleep(1000);
            //robot.bckWithTime(0.3, 200, this);
            robot.killAll();
            robot.resetEncoders();
            robot.fwdWithEncoder(1, 850, this);
            robot.turnLeft180(1,this);
            robot.killAll();
            robot.resetEncoders();
            robot.unclampBack();
            sleep(1000);
            robot.fwdWithEncoderNoCorrect(0.6, 200, this);
            robot.resetEncoders();
            robot.turnRightEnc(0.6, 600, this);
            robot.turnRightGyro(90, this);
            robot.killAll();
            robot.resetEncoders();
            robot.strafeLeftWithEnc(0.9, 5650, this);
            robot.reOrient(this);
            robot.killAll();
            robot.setMaintainedHeading(0);
            robot.resetEncoders();
            robot.strafeRightWithEnc(0.6, 200, this);
            robot.killAll();
            robot.resetEncoders();
            robot.grabBack();
            sleep(800);
            robot.strafeLeftWithEnc(0.6, 450, this);
            robot.killAll();
            robot.resetEncoders();
            robot.turnLeftGyro(80, this);
            robot.killAll();
            robot.setMaintainedHeading(85);
            robot.killAll();
            robot.resetEncoders();
            robot.strafeRightWithEnc(1, 4500, this);
            robot.killAll();
            robot.resetEncoders();
            robot.releaseBack();
            sleep(1000);
            robot.strafeLeftWithEnc(1, 1000, this);
            robot.killAll();
            robot.resetEncoders();
            robot.reOrient(this);
            robot.killAll();
            robot.leftArm.setPosition(1);
            robot.rightArm.setPosition(0);
            sleep(2000);

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
            robot.strafeLeftWithEnc(0.6, 350, this);
            robot.killAll();
            robot.resetEncoders();
            robot.turnLeftGyro(80, this);
            robot.killAll();
            robot.setMaintainedHeading(85);
            robot.killAll();
            robot.resetEncoders();
            robot.strafeRightWithEnc(0.8, 4500, this);
            robot.releaseFront();
            robot.strafeRightWithEnc(0.68, 6000, this);
            //robot.strafeRightWithTime(0.2, 2000, this);
            robot.resetEncoders();
            robot.setMaintainedHeading(90);
            //robot.strafeLeftWithEnc(0.3, 900, this);
            robot.killAll();
            robot.resetEncoders();
            //robot.strafeRightWithEnc(0.3, 800, this);
            robot.resetEncoders();
            robot.bckWithTime(0.3, 1000, this);
            robot.killAll();
            robot.resetEncoders();
            robot.clampBack();
            sleep(1000);
            //robot.bckWithTime(0.3, 200, this);
            robot.killAll();
            robot.resetEncoders();
            robot.fwdWithEncoder(1, 850, this);
            robot.turnLeft180(1,this);
            robot.killAll();
            robot.resetEncoders();
            robot.unclampBack();
            sleep(1000);
            robot.fwdWithEncoderNoCorrect(0.6, 200, this);
            robot.resetEncoders();
            robot.turnRightEnc(0.6, 600, this);
            robot.turnRightGyro(90, this);
            robot.killAll();
            robot.resetEncoders();
            robot.strafeLeftWithEnc(0.9, 6250, this);
            robot.reOrient(this);
            robot.killAll();
            robot.setMaintainedHeading(0);
            robot.resetEncoders();
            robot.strafeRightWithEnc(0.6, 200, this);
            robot.killAll();
            robot.resetEncoders();
            robot.grabBack();
            sleep(800);
            robot.strafeLeftWithEnc(0.6, 400, this);
            robot.killAll();
            robot.resetEncoders();
            robot.turnLeftGyro(80, this);
            robot.killAll();
            robot.setMaintainedHeading(85);
            robot.killAll();
            robot.resetEncoders();
            robot.strafeRightWithEnc(1, 4500, this);
            robot.killAll();
            robot.resetEncoders();
            robot.releaseBack();
            sleep(1000);
            robot.strafeLeftWithEnc(1, 1000, this);
            robot.killAll();
            robot.resetEncoders();
            robot.turnLeft180(0.6,this);
            robot.killAll();
            robot.leftArm.setPosition(1);
            robot.rightArm.setPosition(0);
            sleep(2000);

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
            robot.strafeLeftWithEnc(0.6, 420, this);
            robot.killAll();
            robot.resetEncoders();
            robot.turnLeftGyro(80, this);
            robot.killAll();
            robot.setMaintainedHeading(85);
            robot.killAll();
            robot.resetEncoders();
            robot.strafeRightWithEnc(0.8, 4500, this);
            robot.releaseBack();
            robot.strafeRightWithEnc(0.68, 6080, this);
            //robot.strafeRightWithTime(0.2, 2000, this);
            robot.resetEncoders();
            robot.setMaintainedHeading(90);
            //robot.strafeLeftWithEnc(0.3, 900, this);
            robot.killAll();
            robot.resetEncoders();
            //robot.strafeRightWithEnc(0.3, 800, this);
            robot.resetEncoders();
            robot.bckWithTime(0.3, 1000, this);
            robot.killAll();
            robot.resetEncoders();
            robot.clampBack();
            sleep(1000);
            //robot.bckWithTime(0.3, 200, this);
            robot.killAll();
            robot.resetEncoders();
            robot.fwdWithEncoder(1, 850, this);
            robot.turnLeft180(1,this);
            robot.killAll();
            robot.resetEncoders();
            robot.unclampBack();
            sleep(1000);
            robot.fwdWithEncoderNoCorrect(0.6, 200, this);
            robot.resetEncoders();
            robot.turnRightEnc(0.6, 600, this);
            robot.turnRightGyro(90, this);
            robot.killAll();
            robot.resetEncoders();
            robot.strafeLeftWithEnc(0.9, 6750, this);
            robot.reOrient(this);
            robot.killAll();
            robot.setMaintainedHeading(0);
            robot.resetEncoders();
            robot.bckWithEncoder(0.6, 200, this);
            robot.killAll();
            robot.resetEncoders();
            robot.strafeRightWithEnc(0.6, 200, this);
            robot.killAll();
            robot.resetEncoders();
            robot.grabBack();
            sleep(800);
            robot.strafeLeftWithEnc(0.6, 400, this);
            robot.killAll();
            robot.resetEncoders();
            robot.turnLeftGyro(80, this);
            robot.killAll();
            robot.setMaintainedHeading(85);
            robot.killAll();
            robot.resetEncoders();
            robot.strafeRightWithEnc(1, 4500, this);
            robot.killAll();
            robot.resetEncoders();
            robot.releaseBack();
            sleep(1000);
            robot.strafeLeftWithEnc(1, 1000, this);
            robot.killAll();
            robot.resetEncoders();
            robot.reOrient(this);
            robot.killAll();
            robot.leftArm.setPosition(1);
            robot.rightArm.setPosition(0);
            sleep(2000);

        }

    }
}

