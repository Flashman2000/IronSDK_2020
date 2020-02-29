package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.configs.Field_Locations;
import org.firstinspires.ftc.teamcode.drive.localizer.StandardThreeWheelLocalizer;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;
import org.firstinspires.ftc.teamcode.util.Pose;
import org.opencv.core.Mat;

@Autonomous
public class New_Red extends LinearOpMode {

    String SkystoneLocation;
    String grab = "Grab";
    String drop = "Drop";

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDriveREVOptimized drive = new SampleMecanumDriveREVOptimized(hardwareMap, true);

        StandardThreeWheelLocalizer odometers = new StandardThreeWheelLocalizer(hardwareMap);

        Field_Locations locations = new Field_Locations();

        ConstantInterpolator FACING_LZ = new ConstantInterpolator(Math.toRadians(0));


        drive.backs.setPosition(1);

        drive.frontYk.setPosition(0);
        drive.frontYkA.setPosition(0.5);

        //drive.frontYkA.setPosition(1);

        while (!isStarted()) {

            SkystoneLocation = drive.detectSkystone();
            telemetry.addData("Skystone Location", SkystoneLocation);
            telemetry.update();

            if (opModeIsActive()) {
                break;
            }

        }

        double oldOdometerPosL = odometers.leftEncoder.getCurrentPosition();
        double oldOdometerPosR = odometers.rightEncoder.getCurrentPosition();
        double oldOdometerPosB = odometers.backEncoder.getCurrentPosition();


        telemetry.addLine("Ready");
        telemetry.update();

        SkystoneLocation = drive.detectSkystone();

        waitForStart();

        if (SkystoneLocation == "Right" || SkystoneLocation == "NOT DETECTED") {

            drive.setMaintainedHeading(0);

            //Align with 3rd block
            drive.fwdWithOdo(odometers.leftEncoder, odometers.rightEncoder,
                    4000, oldOdometerPosL, oldOdometerPosR, this);

            drive.killAll();

            /*
            double offset = drive.imu.getAngularOrientation().firstAngle;
            drive.turnSync(Math.toRadians(-offset), drive.RF, drive.LB);

             */

            //lower claw
            drive.frontYk.setPosition(0);
            drive.frontYkA.setPosition(0.8);

            //sleep(50000);


            //reset enc pos
            oldOdometerPosL = odometers.leftEncoder.getCurrentPosition();
            oldOdometerPosR = odometers.rightEncoder.getCurrentPosition();
            oldOdometerPosB = odometers.backEncoder.getCurrentPosition();

            //strafe to pickup first skystone
            drive.strafeRightWithOdo(odometers.backEncoder, 41000, oldOdometerPosB, this);

            drive.killAll();

            //grab the 1st skystone
            drive.frontYkA.setPosition(1);
            sleep(200);
            drive.frontYk.setPosition(1);
            drive.frontYk.setPosition(0.5);
            sleep(500);
            drive.frontYkA.setPosition(0.6);

            //fix offset
            /*
            offset = drive.imu.getAngularOrientation().firstAngle;
            drive.turnSync(-offset, drive.RF, drive.LB);

             */

            //reset encoder pos
            oldOdometerPosL = odometers.leftEncoder.getCurrentPosition();
            oldOdometerPosR = odometers.rightEncoder.getCurrentPosition();
            oldOdometerPosB = odometers.backEncoder.getCurrentPosition();

            //strafe left to drop first skystone
            drive.strafeLeftWithOdo(odometers.backEncoder, 4500, oldOdometerPosB, this);

            drive.killAll();

            /*
            offset = drive.imu.getAngularOrientation().firstAngle;
            drive.turnSync(Math.toRadians(-offset), drive.RF, drive.LB);
            offset = drive.imu.getAngularOrientation().firstAngle;
            drive.turnSync(Math.toRadians(-offset), drive.RF, drive.LB);
            offset = drive.imu.getAngularOrientation().firstAngle;
            drive.turnSync(Math.toRadians(-offset), drive.RF, drive.LB);

             */

            sleep(150);

            oldOdometerPosL = odometers.leftEncoder.getCurrentPosition();
            oldOdometerPosR = odometers.rightEncoder.getCurrentPosition();
            oldOdometerPosB = odometers.backEncoder.getCurrentPosition();

            drive.backWithOdo(odometers.leftEncoder, odometers.rightEncoder,
                    148000, oldOdometerPosL, oldOdometerPosR, this);

            drive.killAll();

            /*
            offset = drive.imu.getAngularOrientation().firstAngle;
            drive.turnSync(Math.toRadians(-offset), drive.RF, drive.LB);

             */

            sleep(150);

            oldOdometerPosL = odometers.leftEncoder.getCurrentPosition();
            oldOdometerPosR = odometers.rightEncoder.getCurrentPosition();
            oldOdometerPosB = odometers.backEncoder.getCurrentPosition();

            drive.strafeRightWithOdo(odometers.backEncoder, 8000, oldOdometerPosB, this);

            drive.killAll();

            /*
            offset = drive.imu.getAngularOrientation().firstAngle;
            drive.turnSync(Math.toRadians(-offset), drive.RF, drive.LB);

             */

            drive.frontYkA.setPosition(1);
            sleep(200);
            drive.frontYk.setPosition(0);
            sleep(200);
            drive.frontYkA.setPosition(0.6);
            sleep(100);
            drive.frontYk.setPosition(0.75);

            oldOdometerPosL = odometers.leftEncoder.getCurrentPosition();
            oldOdometerPosR = odometers.rightEncoder.getCurrentPosition();
            oldOdometerPosB = odometers.backEncoder.getCurrentPosition();

            drive.strafeLeftWithOdo(odometers.backEncoder, 5700, oldOdometerPosB, this);

            drive.killAll();

            sleep(150);

            oldOdometerPosL = odometers.leftEncoder.getCurrentPosition();
            oldOdometerPosR = odometers.rightEncoder.getCurrentPosition();
            oldOdometerPosB = odometers.backEncoder.getCurrentPosition();

            drive.fwdWithOdo(odometers.leftEncoder, odometers.rightEncoder,
                    108000, oldOdometerPosL, oldOdometerPosR, this);

            drive.killAll();


            drive.frontYk.setPosition(0);

            sleep(250);

            oldOdometerPosL = odometers.leftEncoder.getCurrentPosition();
            oldOdometerPosR = odometers.rightEncoder.getCurrentPosition();
            oldOdometerPosB = odometers.backEncoder.getCurrentPosition();

            drive.strafeRightWithOdo(odometers.backEncoder, 2500, oldOdometerPosB, this);

            drive.killAll();

            drive.frontYkA.setPosition(1);
            sleep(200);
            drive.frontYk.setPosition(1);
            drive.frontYk.setPosition(0.5);
            sleep(500);
            drive.frontYkA.setPosition(0.6);

            oldOdometerPosL = odometers.leftEncoder.getCurrentPosition();
            oldOdometerPosR = odometers.rightEncoder.getCurrentPosition();
            oldOdometerPosB = odometers.backEncoder.getCurrentPosition();

            drive.strafeLeftWithOdo(odometers.backEncoder, 2700, oldOdometerPosB, this);

            drive.killAll();

            sleep(150);

            drive.backWithOdo(odometers.leftEncoder, odometers.rightEncoder,
                    122000, oldOdometerPosL, oldOdometerPosR, this);

            drive.killAll();

            sleep(150);
            oldOdometerPosL = odometers.leftEncoder.getCurrentPosition();
            oldOdometerPosR = odometers.rightEncoder.getCurrentPosition();
            oldOdometerPosB = odometers.backEncoder.getCurrentPosition();

            drive.strafeRightWithOdo(odometers.backEncoder, 5600, oldOdometerPosB, this);

            drive.killAll();

            drive.frontYkA.setPosition(1);
            sleep(200);
            drive.frontYk.setPosition(0);
            sleep(200);
            drive.frontYkA.setPosition(0.6);
            sleep(100);
            drive.frontYk.setPosition(0.75);

            oldOdometerPosL = odometers.leftEncoder.getCurrentPosition();
            oldOdometerPosR = odometers.rightEncoder.getCurrentPosition();
            oldOdometerPosB = odometers.backEncoder.getCurrentPosition();

            drive.strafeLeftWithOdo(odometers.backEncoder, 6400, oldOdometerPosB, this);

            drive.killAll();

            sleep(150);

            oldOdometerPosL = odometers.leftEncoder.getCurrentPosition();
            oldOdometerPosR = odometers.rightEncoder.getCurrentPosition();
            oldOdometerPosB = odometers.backEncoder.getCurrentPosition();

            drive.fwdWithOdo(odometers.leftEncoder, odometers.rightEncoder,
                    137000, oldOdometerPosL, oldOdometerPosR, this);

            drive.killAll();

            sleep(150);

            drive.frontYk.setPosition(0);

            sleep(150);

            oldOdometerPosL = odometers.leftEncoder.getCurrentPosition();
            oldOdometerPosR = odometers.rightEncoder.getCurrentPosition();
            oldOdometerPosB = odometers.backEncoder.getCurrentPosition();

            drive.strafeRightWithOdo(odometers.backEncoder, 3100, oldOdometerPosB, this);

            drive.killAll();

            drive.frontYkA.setPosition(1);
            sleep(200);
            drive.frontYk.setPosition(1);
            drive.frontYk.setPosition(0.5);
            sleep(500);
            drive.frontYkA.setPosition(0.6);

            oldOdometerPosL = odometers.leftEncoder.getCurrentPosition();
            oldOdometerPosR = odometers.rightEncoder.getCurrentPosition();
            oldOdometerPosB = odometers.backEncoder.getCurrentPosition();

            drive.strafeLeftWithOdo(odometers.backEncoder, 2600, oldOdometerPosB, this);

            drive.killAll();

            sleep(150);

            drive.backWithOdo(odometers.leftEncoder, odometers.rightEncoder,
                    131500, oldOdometerPosL, oldOdometerPosR, this);

            drive.killAll();

            sleep(150);
            oldOdometerPosL = odometers.leftEncoder.getCurrentPosition();
            oldOdometerPosR = odometers.rightEncoder.getCurrentPosition();
            oldOdometerPosB = odometers.backEncoder.getCurrentPosition();

            drive.strafeRightWithOdo(odometers.backEncoder, 5500, oldOdometerPosB, this);

            drive.killAll();


            drive.backs.setPosition(1);
            drive.frontYkA.setPosition(1);
            sleep(200);
            drive.frontYk.setPosition(0);
            sleep(200);
            drive.frontYkA.setPosition(0.6);
            sleep(100);
            drive.frontYk.setPosition(0.75);

            oldOdometerPosL = odometers.leftEncoder.getCurrentPosition();
            oldOdometerPosR = odometers.rightEncoder.getCurrentPosition();
            oldOdometerPosB = odometers.backEncoder.getCurrentPosition();

            drive.strafeLeftWithOdo(odometers.backEncoder, 6400, oldOdometerPosB, this);

            drive.killAll();

            sleep(250);

            oldOdometerPosL = odometers.leftEncoder.getCurrentPosition();
            oldOdometerPosR = odometers.rightEncoder.getCurrentPosition();
            oldOdometerPosB = odometers.backEncoder.getCurrentPosition();

            drive.turnLeftWithOdo(odometers.backEncoder, 14150, oldOdometerPosB, this);

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .back(13)
                            .build()
            );

            //TODO: clamp code
            drive.backs.setPosition(0);
            sleep(500);

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .forward(30)
                            .build()
            );

            while (drive.imu.getAngularOrientation().firstAngle < 180 && drive.imu.getAngularOrientation().firstAngle > 0 && opModeIsActive()) {
                //double pwr = Range.clip((Math.pow(target,2) - Math.pow(imu.getAngularOrientation().firstAngle,2) / Math.pow(target,2)), 0.3, 1);
                //double pwr = Range.clip(Math.abs(0.011*(drive.imu.getAngularOrientation().firstAngle - Math.toRadians(180))), 0.3, 1);

                if (drive.imu.getAngularOrientation().firstAngle > 60) {
                    drive.frontYkA.setPosition(0.9);
                }

                drive.LF.setPower(0.7);
                drive.LB.setPower(-0.7);
                drive.RF.setPower(0.7);
                drive.RB.setPower(-0.7);
                telemetry.addData("Heading", drive.imu.getAngularOrientation().firstAngle);
                telemetry.update();
            }
            drive.killAll();


            //TODO: unclamp code

            drive.backs.setPosition(1);

            //drive.setMaintainedHeading(180);

            oldOdometerPosL = odometers.leftEncoder.getCurrentPosition();
            oldOdometerPosR = odometers.rightEncoder.getCurrentPosition();
            oldOdometerPosB = odometers.backEncoder.getCurrentPosition();

            drive.strafeRightWithOdo(odometers.backEncoder, 2000, oldOdometerPosB, this);

            drive.leftArm.setPosition(1);
            drive.rightArm.setPosition(0);

            sleep(250);

            oldOdometerPosL = odometers.leftEncoder.getCurrentPosition();
            oldOdometerPosR = odometers.rightEncoder.getCurrentPosition();
            oldOdometerPosB = odometers.backEncoder.getCurrentPosition();

            drive.frontYkA.setPosition(0.6);

            drive.fwdWithOdo(odometers.leftEncoder, odometers.rightEncoder,
                    42000, oldOdometerPosL, oldOdometerPosR, this);

            //drive.strafeLeft(1);


        }

        if (SkystoneLocation == "Center") {

            drive.setMaintainedHeading(0);

            //Align with 3rd block
            drive.fwdWithOdo(odometers.leftEncoder, odometers.rightEncoder,
                    14500, oldOdometerPosL, oldOdometerPosR, this);

            drive.killAll();

            /*
            double offset = drive.imu.getAngularOrientation().firstAngle;
            drive.turnSync(Math.toRadians(-offset), drive.RF, drive.LB);

             */

            //lower claw
            drive.frontYk.setPosition(0);
            drive.frontYkA.setPosition(0.8);


            //reset enc pos
            oldOdometerPosL = odometers.leftEncoder.getCurrentPosition();
            oldOdometerPosR = odometers.rightEncoder.getCurrentPosition();
            oldOdometerPosB = odometers.backEncoder.getCurrentPosition();

            //strafe to pickup first skystone
            drive.strafeRightWithOdo(odometers.backEncoder, 41000, oldOdometerPosB, this);

            drive.killAll();

            //grab the 1st skystone
            drive.frontYkA.setPosition(1);
            sleep(200);
            drive.frontYk.setPosition(1);
            drive.frontYk.setPosition(0.5);
            sleep(500);
            drive.frontYkA.setPosition(0.6);

            //fix offset
            /*
            offset = drive.imu.getAngularOrientation().firstAngle;
            drive.turnSync(-offset, drive.RF, drive.LB);

             */

            //reset encoder pos
            oldOdometerPosL = odometers.leftEncoder.getCurrentPosition();
            oldOdometerPosR = odometers.rightEncoder.getCurrentPosition();
            oldOdometerPosB = odometers.backEncoder.getCurrentPosition();

            //strafe left to drop first skystone
            drive.strafeLeftWithOdo(odometers.backEncoder, 4500, oldOdometerPosB, this);

            drive.killAll();

            /*
            offset = drive.imu.getAngularOrientation().firstAngle;
            drive.turnSync(Math.toRadians(-offset), drive.RF, drive.LB);
            offset = drive.imu.getAngularOrientation().firstAngle;
            drive.turnSync(Math.toRadians(-offset), drive.RF, drive.LB);
            offset = drive.imu.getAngularOrientation().firstAngle;
            drive.turnSync(Math.toRadians(-offset), drive.RF, drive.LB);

             */

            sleep(150);

            oldOdometerPosL = odometers.leftEncoder.getCurrentPosition();
            oldOdometerPosR = odometers.rightEncoder.getCurrentPosition();
            oldOdometerPosB = odometers.backEncoder.getCurrentPosition();

            drive.backWithOdo(odometers.leftEncoder, odometers.rightEncoder,
                    163000, oldOdometerPosL, oldOdometerPosR, this);

            drive.killAll();

            /*
            offset = drive.imu.getAngularOrientation().firstAngle;
            drive.turnSync(Math.toRadians(-offset), drive.RF, drive.LB);

             */

            sleep(150);

            oldOdometerPosL = odometers.leftEncoder.getCurrentPosition();
            oldOdometerPosR = odometers.rightEncoder.getCurrentPosition();
            oldOdometerPosB = odometers.backEncoder.getCurrentPosition();

            drive.strafeRightWithOdo(odometers.backEncoder, 8700, oldOdometerPosB, this);

            drive.killAll();

            /*
            offset = drive.imu.getAngularOrientation().firstAngle;
            drive.turnSync(Math.toRadians(-offset), drive.RF, drive.LB);

             */

            drive.frontYkA.setPosition(1);
            sleep(200);
            drive.frontYk.setPosition(0);
            sleep(200);
            drive.frontYkA.setPosition(0.6);
            sleep(100);
            drive.frontYk.setPosition(0.75);

            oldOdometerPosL = odometers.leftEncoder.getCurrentPosition();
            oldOdometerPosR = odometers.rightEncoder.getCurrentPosition();
            oldOdometerPosB = odometers.backEncoder.getCurrentPosition();

            drive.strafeLeftWithOdo(odometers.backEncoder, 6400, oldOdometerPosB, this);

            drive.killAll();

            sleep(150);

            oldOdometerPosL = odometers.leftEncoder.getCurrentPosition();
            oldOdometerPosR = odometers.rightEncoder.getCurrentPosition();
            oldOdometerPosB = odometers.backEncoder.getCurrentPosition();

            drive.fwdWithOdo(odometers.leftEncoder, odometers.rightEncoder,
                    118000, oldOdometerPosL, oldOdometerPosR, this);

            drive.killAll();


            drive.frontYk.setPosition(0);

            sleep(250);

            oldOdometerPosL = odometers.leftEncoder.getCurrentPosition();
            oldOdometerPosR = odometers.rightEncoder.getCurrentPosition();
            oldOdometerPosB = odometers.backEncoder.getCurrentPosition();

            drive.strafeRightWithOdo(odometers.backEncoder, 3400, oldOdometerPosB, this);

            drive.killAll();

            drive.frontYkA.setPosition(1);
            sleep(200);
            drive.frontYk.setPosition(1);
            drive.frontYk.setPosition(0.5);
            sleep(500);
            drive.frontYkA.setPosition(0.6);

            oldOdometerPosL = odometers.leftEncoder.getCurrentPosition();
            oldOdometerPosR = odometers.rightEncoder.getCurrentPosition();
            oldOdometerPosB = odometers.backEncoder.getCurrentPosition();

            drive.strafeLeftWithOdo(odometers.backEncoder, 3700, oldOdometerPosB, this);

            drive.killAll();

            sleep(150);

            drive.backWithOdo(odometers.leftEncoder, odometers.rightEncoder,
                    137000, oldOdometerPosL, oldOdometerPosR, this);

            drive.killAll();

            sleep(150);
            oldOdometerPosL = odometers.leftEncoder.getCurrentPosition();
            oldOdometerPosR = odometers.rightEncoder.getCurrentPosition();
            oldOdometerPosB = odometers.backEncoder.getCurrentPosition();

            drive.strafeRightWithOdo(odometers.backEncoder, 6000, oldOdometerPosB, this);

            drive.killAll();

            drive.frontYkA.setPosition(1);
            sleep(200);
            drive.frontYk.setPosition(0);
            sleep(200);
            drive.frontYkA.setPosition(0.6);
            sleep(100);
            drive.frontYk.setPosition(0.75);

            oldOdometerPosL = odometers.leftEncoder.getCurrentPosition();
            oldOdometerPosR = odometers.rightEncoder.getCurrentPosition();
            oldOdometerPosB = odometers.backEncoder.getCurrentPosition();

            drive.strafeLeftWithOdo(odometers.backEncoder, 6800, oldOdometerPosB, this);

            drive.killAll();

            sleep(150);

            oldOdometerPosL = odometers.leftEncoder.getCurrentPosition();
            oldOdometerPosR = odometers.rightEncoder.getCurrentPosition();
            oldOdometerPosB = odometers.backEncoder.getCurrentPosition();

            drive.fwdWithOdo(odometers.leftEncoder, odometers.rightEncoder,
                    122000, oldOdometerPosL, oldOdometerPosR, this);

            drive.killAll();

            sleep(150);

            drive.frontYk.setPosition(0);

            sleep(150);

            oldOdometerPosL = odometers.leftEncoder.getCurrentPosition();
            oldOdometerPosR = odometers.rightEncoder.getCurrentPosition();
            oldOdometerPosB = odometers.backEncoder.getCurrentPosition();

            drive.strafeRightWithOdo(odometers.backEncoder, 5300, oldOdometerPosB, this);

            drive.killAll();

            drive.frontYkA.setPosition(1);
            sleep(200);
            drive.frontYk.setPosition(1);
            drive.frontYk.setPosition(0.5);
            sleep(500);
            drive.frontYkA.setPosition(0.6);

            oldOdometerPosL = odometers.leftEncoder.getCurrentPosition();
            oldOdometerPosR = odometers.rightEncoder.getCurrentPosition();
            oldOdometerPosB = odometers.backEncoder.getCurrentPosition();

            drive.strafeLeftWithOdo(odometers.backEncoder, 3800, oldOdometerPosB, this);

            drive.killAll();

            sleep(150);

            drive.backWithOdo(odometers.leftEncoder, odometers.rightEncoder,
                    120000, oldOdometerPosL, oldOdometerPosR, this);

            drive.killAll();

            sleep(150);
            oldOdometerPosL = odometers.leftEncoder.getCurrentPosition();
            oldOdometerPosR = odometers.rightEncoder.getCurrentPosition();
            oldOdometerPosB = odometers.backEncoder.getCurrentPosition();

            drive.strafeRightWithOdo(odometers.backEncoder, 5500, oldOdometerPosB, this);

            drive.killAll();


            drive.backs.setPosition(1);
            drive.frontYkA.setPosition(1);
            sleep(200);
            drive.frontYk.setPosition(0);
            sleep(200);
            drive.frontYkA.setPosition(0.6);
            sleep(100);
            drive.frontYk.setPosition(0.75);

            oldOdometerPosL = odometers.leftEncoder.getCurrentPosition();
            oldOdometerPosR = odometers.rightEncoder.getCurrentPosition();
            oldOdometerPosB = odometers.backEncoder.getCurrentPosition();

            drive.strafeLeftWithOdo(odometers.backEncoder, 6400, oldOdometerPosB, this);

            drive.killAll();

            sleep(250);

            oldOdometerPosL = odometers.leftEncoder.getCurrentPosition();
            oldOdometerPosR = odometers.rightEncoder.getCurrentPosition();
            oldOdometerPosB = odometers.backEncoder.getCurrentPosition();

            drive.turnLeftWithOdo(odometers.backEncoder, 14000, oldOdometerPosB, this);

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .back(13)
                            .build()
            );

            //TODO: clamp code
            drive.backs.setPosition(0);
            sleep(500);

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .forward(30)
                            .build()
            );

            while (drive.imu.getAngularOrientation().firstAngle < 180 && drive.imu.getAngularOrientation().firstAngle > 0 && opModeIsActive()) {
                //double pwr = Range.clip((Math.pow(target,2) - Math.pow(imu.getAngularOrientation().firstAngle,2) / Math.pow(target,2)), 0.3, 1);
                //double pwr = Range.clip(Math.abs(0.011*(drive.imu.getAngularOrientation().firstAngle - Math.toRadians(180))), 0.3, 1);

                if (drive.imu.getAngularOrientation().firstAngle > 60) {
                    drive.frontYkA.setPosition(0.9);
                }

                drive.LF.setPower(0.7);
                drive.LB.setPower(-0.7);
                drive.RF.setPower(0.7);
                drive.RB.setPower(-0.7);
                telemetry.addData("Heading", drive.imu.getAngularOrientation().firstAngle);
                telemetry.update();
            }
            drive.killAll();


            //TODO: unclamp code

            drive.backs.setPosition(1);

            //drive.setMaintainedHeading(180);

            oldOdometerPosL = odometers.leftEncoder.getCurrentPosition();
            oldOdometerPosR = odometers.rightEncoder.getCurrentPosition();
            oldOdometerPosB = odometers.backEncoder.getCurrentPosition();

            drive.strafeRightWithOdo(odometers.backEncoder, 2000, oldOdometerPosB, this);

            drive.leftArm.setPosition(1);
            drive.rightArm.setPosition(0);

            sleep(250);

            oldOdometerPosL = odometers.leftEncoder.getCurrentPosition();
            oldOdometerPosR = odometers.rightEncoder.getCurrentPosition();
            oldOdometerPosB = odometers.backEncoder.getCurrentPosition();

            drive.frontYkA.setPosition(0.6);

            drive.fwdWithOdo(odometers.leftEncoder, odometers.rightEncoder,
                    40000, oldOdometerPosL, oldOdometerPosR, this);

            //drive.strafeLeft(1);

        }

        if (SkystoneLocation == "Left") {

            drive.setMaintainedHeading(0);

            //Align with 3rd block
            drive.fwdWithOdo(odometers.leftEncoder, odometers.rightEncoder,
                    27000, oldOdometerPosL, oldOdometerPosR, this);

            drive.killAll();

            /*
            double offset = drive.imu.getAngularOrientation().firstAngle;
            drive.turnSync(Math.toRadians(-offset), drive.RF, drive.LB);

             */

            //lower claw
            drive.frontYk.setPosition(0);
            drive.frontYkA.setPosition(0.8);


            //reset enc pos
            oldOdometerPosL = odometers.leftEncoder.getCurrentPosition();
            oldOdometerPosR = odometers.rightEncoder.getCurrentPosition();
            oldOdometerPosB = odometers.backEncoder.getCurrentPosition();

            //strafe to pickup first skystone
            drive.strafeRightWithOdo(odometers.backEncoder, 41000, oldOdometerPosB, this);

            drive.killAll();

            //grab the 1st skystone
            drive.frontYkA.setPosition(1);
            sleep(200);
            drive.frontYk.setPosition(1);
            drive.frontYk.setPosition(0.5);
            sleep(500);
            drive.frontYkA.setPosition(0.6);

            //fix offset
            /*
            offset = drive.imu.getAngularOrientation().firstAngle;
            drive.turnSync(-offset, drive.RF, drive.LB);

             */

            //reset encoder pos
            oldOdometerPosL = odometers.leftEncoder.getCurrentPosition();
            oldOdometerPosR = odometers.rightEncoder.getCurrentPosition();
            oldOdometerPosB = odometers.backEncoder.getCurrentPosition();

            //strafe left to drop first skystone
            drive.strafeLeftWithOdo(odometers.backEncoder, 4100, oldOdometerPosB, this);

            drive.killAll();

            /*
            offset = drive.imu.getAngularOrientation().firstAngle;
            drive.turnSync(Math.toRadians(-offset), drive.RF, drive.LB);
            offset = drive.imu.getAngularOrientation().firstAngle;
            drive.turnSync(Math.toRadians(-offset), drive.RF, drive.LB);
            offset = drive.imu.getAngularOrientation().firstAngle;
            drive.turnSync(Math.toRadians(-offset), drive.RF, drive.LB);

             */

            sleep(150);

            oldOdometerPosL = odometers.leftEncoder.getCurrentPosition();
            oldOdometerPosR = odometers.rightEncoder.getCurrentPosition();
            oldOdometerPosB = odometers.backEncoder.getCurrentPosition();

            drive.backWithOdo(odometers.leftEncoder, odometers.rightEncoder,
                    174500, oldOdometerPosL, oldOdometerPosR, this);

            drive.killAll();

            /*
            offset = drive.imu.getAngularOrientation().firstAngle;
            drive.turnSync(Math.toRadians(-offset), drive.RF, drive.LB);

             */

            sleep(150);

            oldOdometerPosL = odometers.leftEncoder.getCurrentPosition();
            oldOdometerPosR = odometers.rightEncoder.getCurrentPosition();
            oldOdometerPosB = odometers.backEncoder.getCurrentPosition();

            drive.strafeRightWithOdo(odometers.backEncoder, 8000, oldOdometerPosB, this);

            drive.killAll();

            /*
            offset = drive.imu.getAngularOrientation().firstAngle;
            drive.turnSync(Math.toRadians(-offset), drive.RF, drive.LB);

             */

            drive.frontYkA.setPosition(1);
            sleep(200);
            drive.frontYk.setPosition(0);
            sleep(200);
            drive.frontYkA.setPosition(0.6);
            sleep(100);
            drive.frontYk.setPosition(0.75);

            oldOdometerPosL = odometers.leftEncoder.getCurrentPosition();
            oldOdometerPosR = odometers.rightEncoder.getCurrentPosition();
            oldOdometerPosB = odometers.backEncoder.getCurrentPosition();

            drive.strafeLeftWithOdo(odometers.backEncoder, 5700, oldOdometerPosB, this);

            drive.killAll();

            sleep(150);

            oldOdometerPosL = odometers.leftEncoder.getCurrentPosition();
            oldOdometerPosR = odometers.rightEncoder.getCurrentPosition();
            oldOdometerPosB = odometers.backEncoder.getCurrentPosition();

            drive.fwdWithOdo(odometers.leftEncoder, odometers.rightEncoder,
                    134000, oldOdometerPosL, oldOdometerPosR, this);

            drive.killAll();


            drive.frontYk.setPosition(0);

            sleep(250);

            oldOdometerPosL = odometers.leftEncoder.getCurrentPosition();
            oldOdometerPosR = odometers.rightEncoder.getCurrentPosition();
            oldOdometerPosB = odometers.backEncoder.getCurrentPosition();

            drive.strafeRightWithOdo(odometers.backEncoder, 2500, oldOdometerPosB, this);

            drive.killAll();

            drive.frontYkA.setPosition(1);
            sleep(200);
            drive.frontYk.setPosition(1);
            drive.frontYk.setPosition(0.5);
            sleep(500);
            drive.frontYkA.setPosition(0.6);

            oldOdometerPosL = odometers.leftEncoder.getCurrentPosition();
            oldOdometerPosR = odometers.rightEncoder.getCurrentPosition();
            oldOdometerPosB = odometers.backEncoder.getCurrentPosition();

            drive.strafeLeftWithOdo(odometers.backEncoder, 2700, oldOdometerPosB, this);

            drive.killAll();
//pepega clap you fucking retard make it work 777 auto niggaaaaaaaaaaaaaaaaaaaaaa
            sleep(150);

            drive.backWithOdo(odometers.leftEncoder, odometers.rightEncoder,
                    144500, oldOdometerPosL, oldOdometerPosR, this);

            drive.killAll();

            sleep(150);
            oldOdometerPosL = odometers.leftEncoder.getCurrentPosition();
            oldOdometerPosR = odometers.rightEncoder.getCurrentPosition();
            oldOdometerPosB = odometers.backEncoder.getCurrentPosition();

            drive.strafeRightWithOdo(odometers.backEncoder, 6600, oldOdometerPosB, this);

            drive.killAll();

            drive.frontYkA.setPosition(1);
            sleep(200);
            drive.frontYk.setPosition(0);
            sleep(200);
            drive.frontYkA.setPosition(0.6);
            sleep(100);
            drive.frontYk.setPosition(0.75);

            oldOdometerPosL = odometers.leftEncoder.getCurrentPosition();
            oldOdometerPosR = odometers.rightEncoder.getCurrentPosition();
            oldOdometerPosB = odometers.backEncoder.getCurrentPosition();

            drive.strafeLeftWithOdo(odometers.backEncoder, 7400, oldOdometerPosB, this);

            drive.killAll();

            sleep(150);

            oldOdometerPosL = odometers.leftEncoder.getCurrentPosition();
            oldOdometerPosR = odometers.rightEncoder.getCurrentPosition();
            oldOdometerPosB = odometers.backEncoder.getCurrentPosition();

            drive.fwdWithOdo(odometers.leftEncoder, odometers.rightEncoder,
                    114000, oldOdometerPosL, oldOdometerPosR, this);

            drive.killAll();

            sleep(150);

            drive.frontYk.setPosition(0);

            sleep(150);

            oldOdometerPosL = odometers.leftEncoder.getCurrentPosition();
            oldOdometerPosR = odometers.rightEncoder.getCurrentPosition();
            oldOdometerPosB = odometers.backEncoder.getCurrentPosition();

            drive.strafeRightWithOdo(odometers.backEncoder, 4500, oldOdometerPosB, this);

            drive.killAll();

            drive.frontYkA.setPosition(1);
            sleep(200);
            drive.frontYk.setPosition(1);
            drive.frontYk.setPosition(0.5);
            sleep(500);
            drive.frontYkA.setPosition(0.6);

            oldOdometerPosL = odometers.leftEncoder.getCurrentPosition();
            oldOdometerPosR = odometers.rightEncoder.getCurrentPosition();
            oldOdometerPosB = odometers.backEncoder.getCurrentPosition();

            drive.strafeLeftWithOdo(odometers.backEncoder, 2900, oldOdometerPosB, this);

            drive.killAll();

            sleep(150);

            drive.backWithOdo(odometers.leftEncoder, odometers.rightEncoder,
                    114000, oldOdometerPosL, oldOdometerPosR, this);

            drive.killAll();

            sleep(150);
            oldOdometerPosL = odometers.leftEncoder.getCurrentPosition();
            oldOdometerPosR = odometers.rightEncoder.getCurrentPosition();
            oldOdometerPosB = odometers.backEncoder.getCurrentPosition();

            drive.strafeRightWithOdo(odometers.backEncoder, 5500, oldOdometerPosB, this);

            drive.killAll();


            drive.backs.setPosition(1);
            drive.frontYkA.setPosition(1);
            sleep(200);
            drive.frontYk.setPosition(0);
            sleep(200);
            drive.frontYkA.setPosition(0.6);
            sleep(100);
            drive.frontYk.setPosition(0.75);

            oldOdometerPosL = odometers.leftEncoder.getCurrentPosition();
            oldOdometerPosR = odometers.rightEncoder.getCurrentPosition();
            oldOdometerPosB = odometers.backEncoder.getCurrentPosition();

            drive.strafeLeftWithOdo(odometers.backEncoder, 6400, oldOdometerPosB, this);

            drive.killAll();

            sleep(250);

            oldOdometerPosL = odometers.leftEncoder.getCurrentPosition();
            oldOdometerPosR = odometers.rightEncoder.getCurrentPosition();
            oldOdometerPosB = odometers.backEncoder.getCurrentPosition();

            drive.turnLeftWithOdo(odometers.backEncoder, 14000, oldOdometerPosB, this);

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .back(15)
                            .build()
            );

            //TODO: clamp code
            drive.backs.setPosition(0);
            sleep(500);

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .forward(30)
                            .build()
            );

            while (drive.imu.getAngularOrientation().firstAngle < 180 && drive.imu.getAngularOrientation().firstAngle > 0 && opModeIsActive()) {
                //double pwr = Range.clip((Math.pow(target,2) - Math.pow(imu.getAngularOrientation().firstAngle,2) / Math.pow(target,2)), 0.3, 1);
                //double pwr = Range.clip(Math.abs(0.011*(drive.imu.getAngularOrientation().firstAngle - Math.toRadians(180))), 0.3, 1);

                if (drive.imu.getAngularOrientation().firstAngle > 60) {
                    drive.frontYkA.setPosition(0.9);
                }

                drive.LF.setPower(0.7);
                drive.LB.setPower(-0.7);
                drive.RF.setPower(0.7);
                drive.RB.setPower(-0.7);
                telemetry.addData("Heading", drive.imu.getAngularOrientation().firstAngle);
                telemetry.update();
            }
            drive.killAll();


            //TODO: unclamp code

            drive.backs.setPosition(1);

            //drive.setMaintainedHeading(180);

            oldOdometerPosL = odometers.leftEncoder.getCurrentPosition();
            oldOdometerPosR = odometers.rightEncoder.getCurrentPosition();
            oldOdometerPosB = odometers.backEncoder.getCurrentPosition();

            drive.strafeRightWithOdo(odometers.backEncoder, 2000, oldOdometerPosB, this);

            drive.leftArm.setPosition(1);
            drive.rightArm.setPosition(0);

            sleep(250);

            oldOdometerPosL = odometers.leftEncoder.getCurrentPosition();
            oldOdometerPosR = odometers.rightEncoder.getCurrentPosition();
            oldOdometerPosB = odometers.backEncoder.getCurrentPosition();

            drive.frontYkA.setPosition(0.6);

            drive.fwdWithOdo(odometers.leftEncoder, odometers.rightEncoder,
                    42000, oldOdometerPosL, oldOdometerPosR, this);

            //drive.strafeLeft(1);

        }

    }
}