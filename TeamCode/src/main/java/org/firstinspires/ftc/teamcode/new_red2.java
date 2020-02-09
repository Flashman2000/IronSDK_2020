package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.configs.Field_Locations;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;
import org.openftc.revextensions2.ExpansionHubEx;

@Autonomous
public class new_red2 extends LinearOpMode {

    String SkystoneLocation;

    @Override
    public void runOpMode() throws InterruptedException{

        SampleMecanumDriveREVOptimized drive = new SampleMecanumDriveREVOptimized(hardwareMap, true);

        double voltage = drive.hub.read12vMonitor(ExpansionHubEx.VoltageUnits.VOLTS);
        double voltage2 = drive.hub2.read12vMonitor(ExpansionHubEx.VoltageUnits.VOLTS);

        Field_Locations locations = new Field_Locations();

        ConstantInterpolator FACING_LZ = new ConstantInterpolator(Math.toRadians(180));

        drive.frontYkA.setPosition(0.75);

        drive.backs.setPosition(1);

        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();

        SkystoneLocation = drive.detectSkystone();

        drive.setPoseEstimate(new Pose2d(-36, -65, Math.toRadians(180)));

        telemetry.addData("Location", SkystoneLocation);
        telemetry.update();

        if(SkystoneLocation == "Left" || SkystoneLocation == "NOT FOUND"){

            drive.frontYkA.setPosition(0.4);
            drive.frontYk.setPosition(0);

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .forward(22)
                            .build()
            );

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .strafeRight(29)
                            .build()

            );


            drive.setPoseEstimate(locations.redSkystone6P);

            //TODO: add grab code

            drive.frontYkA.setPosition(0);
            sleep(200);
            drive.frontYk.setPosition(1);
            drive.backYk.setPosition(0.5);
            sleep(500);
            drive.frontYkA.setPosition(0.65);

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .strafeLeft(8)
                            .build()
            );

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .back(115)
                            .build()
            );

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .strafeRight(8)
                            .build()
            );

            double offset = drive.imu.getAngularOrientation().firstAngle;

            drive.turnSync(-offset, drive.RF, drive.LB);

            drive.frontYkA.setPosition(0);
            sleep(200);
            drive.frontYk.setPosition(0);
            drive.backYk.setPosition(0.5);
            sleep(500);
            drive.frontYkA.setPosition(0.75);


            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .strafeLeft(8)
                            .build()
            );

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .forward(87)
                            .build()
            );


            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .strafeRight(5.5)
                            .build()
            );


            offset = drive.imu.getAngularOrientation().firstAngle;

            drive.turnSync(-offset, drive.RF, drive.LB);

            drive.frontYkA.setPosition(0);
            sleep(500);
            drive.frontYk.setPosition(1);
            drive.backYk.setPosition(0.5);
            sleep(500);
            drive.frontYkA.setPosition(0.65);

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .strafeLeft(6)
                            .build()
            );

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .back(90)
                            .build()
            );

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .strafeRight(6)
                            .build()
            );

            offset = drive.imu.getAngularOrientation().firstAngle;

            drive.turnSync(-offset, drive.RF, drive.LB);

            drive.backs.setPosition(1);

            drive.frontYkA.setPosition(0);
            sleep(200);
            drive.frontYk.setPosition(0);
            drive.backYk.setPosition(0.5);
            sleep(500);
            drive.frontYkA.setPosition(0.75);

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            //.back(5)
                            .strafeLeft(5)
                            .build()
            );

            offset = drive.imu.getAngularOrientation().firstAngle;

            drive.turnSync(-offset, drive.RF, drive.LB);

            drive.turnTo(-90);

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .back(10)
                            .build()
            );

            //TODO: clamp code
            drive.backs.setPosition(0);
            sleep(500);

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .forward(20)
                            .build()
            );

            while (drive.imu.getAngularOrientation().firstAngle < Math.toRadians(180) && drive.imu.getAngularOrientation().firstAngle > Math.toRadians(0) && opModeIsActive()) {
                //double pwr = Range.clip((Math.pow(target,2) - Math.pow(imu.getAngularOrientation().firstAngle,2) / Math.pow(target,2)), 0.3, 1);
                //double pwr = Range.clip(Math.abs(0.011*(drive.imu.getAngularOrientation().firstAngle - Math.toRadians(180))), 0.3, 1);

                if(drive.imu.getAngularOrientation().firstAngle < Math.toRadians(60)){
                    drive.frontYkA.setPosition(0.5);
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

            //TODO: drop front wheels

            drive.leftArm.setPosition(1);
            drive.rightArm.setPosition(0);

            offset = drive.imu.getAngularOrientation().firstAngle;
            drive.backs.setPosition(1);
            drive.turnSync(-offset, drive.RF, drive.LB);

            drive.frontYkA.setPosition(0.75);

            drive.setPoseEstimate(new Pose2d(45, -48, Math.toRadians(179)));

            drive.relayPose(telemetry, drive);

            //TODO: set new pose after empirical analysis

            //sleep(500000);

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            //.strafeRight(2)
                            .forward(32)
                            .build()
            );

        }

        if(SkystoneLocation == "Right"){

            drive.frontYkA.setPosition(0.4);
            drive.frontYk.setPosition(0);

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .forward(4)
                            .strafeRight(36)
                            .build()
            );


            drive.setPoseEstimate(locations.redSkystone4P);

            double offset = drive.imu.getAngularOrientation().firstAngle;

            drive.turnSync(-offset, drive.RF, drive.LB);

            //TODO: add grab code

            drive.frontYkA.setPosition(0);
            sleep(200);
            drive.frontYk.setPosition(1);
            drive.backYk.setPosition(0.5);
            sleep(500);
            drive.frontYkA.setPosition(0.65);

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .strafeLeft(8)
                            .build()
            );

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .back(104)
                            .build()
            );

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .strafeRight(8)
                            .build()
            );

            offset = drive.imu.getAngularOrientation().firstAngle;

            drive.turnSync(-offset, drive.RF, drive.LB);

            drive.frontYkA.setPosition(0);
            sleep(200);
            drive.frontYk.setPosition(0);
            drive.backYk.setPosition(0.5);
            sleep(500);
            drive.frontYkA.setPosition(0.75);


            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .strafeLeft(8)
                            .build()
            );

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .forward(72)
                            .build()
            );


            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .strafeRight(6.5)
                            .build()
            );


            offset = drive.imu.getAngularOrientation().firstAngle;

            drive.turnSync(-offset, drive.RF, drive.LB);

            drive.frontYkA.setPosition(0);
            sleep(500);
            drive.frontYk.setPosition(1);
            drive.backYk.setPosition(0.5);
            sleep(500);
            drive.frontYkA.setPosition(0.65);

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .strafeLeft(6)
                            .build()
            );

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .back(75)
                            .build()
            );

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .strafeRight(6)
                            .build()
            );

            offset = drive.imu.getAngularOrientation().firstAngle;

            drive.turnSync(-offset, drive.RF, drive.LB);

            drive.backs.setPosition(1);

            drive.frontYkA.setPosition(0);
            sleep(200);
            drive.frontYk.setPosition(0);
            drive.backYk.setPosition(0.5);
            sleep(500);
            drive.frontYkA.setPosition(0.75);

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            //.back(5)
                            .strafeLeft(5)
                            .build()
            );

            offset = drive.imu.getAngularOrientation().firstAngle;

            drive.turnSync(-offset, drive.RF, drive.LB);

            drive.turnTo(-90);

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .back(12)
                            .build()
            );

            //TODO: clamp code
            drive.backs.setPosition(0);
            sleep(500);

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .forward(22)
                            .build()
            );

            while (drive.imu.getAngularOrientation().firstAngle < Math.toRadians(180) && drive.imu.getAngularOrientation().firstAngle > Math.toRadians(0) && opModeIsActive()) {
                //double pwr = Range.clip((Math.pow(target,2) - Math.pow(imu.getAngularOrientation().firstAngle,2) / Math.pow(target,2)), 0.3, 1);
                //double pwr = Range.clip(Math.abs(0.011*(drive.imu.getAngularOrientation().firstAngle - Math.toRadians(180))), 0.3, 1);

                if(drive.imu.getAngularOrientation().firstAngle < Math.toRadians(60)){
                    drive.frontYkA.setPosition(0.5);
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

            //TODO: drop front wheels

            drive.leftArm.setPosition(1);
            drive.rightArm.setPosition(0);

            offset = drive.imu.getAngularOrientation().firstAngle;
            drive.backs.setPosition(1);
            drive.turnSync(-offset, drive.RF, drive.LB);

            drive.frontYkA.setPosition(0.75);

            drive.setPoseEstimate(new Pose2d(45, -48, Math.toRadians(179)));

            drive.relayPose(telemetry, drive);

            //TODO: set new pose after empirical analysis

            //sleep(500000);

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .strafeRight(2)
                            .forward(32)
                            .build()
            );

        }

        if(SkystoneLocation == "Center"){

            drive.frontYkA.setPosition(0.4);
            drive.frontYk.setPosition(0);

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .forward(7)
                            .strafeRight(38)
                            .build()
            );


            drive.setPoseEstimate(locations.redSkystone5P);
            double offset = drive.imu.getAngularOrientation().firstAngle;

            drive.turnSync(-offset, drive.RF, drive.LB);

            //TODO: add grab code

            drive.frontYkA.setPosition(0);
            sleep(200);
            drive.frontYk.setPosition(1);
            drive.backYk.setPosition(0.5);
            sleep(500);
            drive.frontYkA.setPosition(0.65);

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                    .strafeLeft(8)
                    .build()
            );

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                    .back(105)
                    .build()
            );

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                    .strafeRight(8)
                    .build()
            );

            offset = drive.imu.getAngularOrientation().firstAngle;

            drive.turnSync(-offset, drive.RF, drive.LB);

            drive.frontYkA.setPosition(0);
            sleep(200);
            drive.frontYk.setPosition(0);
            drive.backYk.setPosition(0.5);
            sleep(500);
            drive.frontYkA.setPosition(0.75);


            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .strafeLeft(8)
                            .build()
            );

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .forward(75)
                            .build()
            );


            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .strafeRight(6)
                            .build()
            );


            offset = drive.imu.getAngularOrientation().firstAngle;

            drive.turnSync(-offset, drive.RF, drive.LB);

            drive.frontYkA.setPosition(0);
            sleep(500);
            drive.frontYk.setPosition(1);
            drive.backYk.setPosition(0.5);
            sleep(500);
            drive.frontYkA.setPosition(0.65);

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .strafeLeft(6)
                            .build()
            );

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .back(90)
                            .build()
            );

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .strafeRight(6)
                            .build()
            );

            offset = drive.imu.getAngularOrientation().firstAngle;

            drive.turnSync(-offset, drive.RF, drive.LB);

            drive.backs.setPosition(1);

            drive.frontYkA.setPosition(0);
            sleep(200);
            drive.frontYk.setPosition(0);
            drive.backYk.setPosition(0.5);
            sleep(500);
            drive.frontYkA.setPosition(0.75);

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                    //.back(5)
                    .strafeLeft(5)
                    .build()
            );

            offset = drive.imu.getAngularOrientation().firstAngle;

            drive.turnSync(-offset, drive.RF, drive.LB);

            drive.turnTo(-90);

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .back(12)
                            .build()
            );

            //TODO: clamp code
            drive.backs.setPosition(0);
            sleep(500);

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .forward(22)
                            .build()
            );

            while (drive.imu.getAngularOrientation().firstAngle < Math.toRadians(180) && drive.imu.getAngularOrientation().firstAngle > Math.toRadians(0) && opModeIsActive()) {
                //double pwr = Range.clip((Math.pow(target,2) - Math.pow(imu.getAngularOrientation().firstAngle,2) / Math.pow(target,2)), 0.3, 1);
                //double pwr = Range.clip(Math.abs(0.011*(drive.imu.getAngularOrientation().firstAngle - Math.toRadians(180))), 0.3, 1);

                if(drive.imu.getAngularOrientation().firstAngle < Math.toRadians(60)){
                    drive.frontYkA.setPosition(0.5);
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

            //TODO: drop front wheels

            drive.leftArm.setPosition(1);
            drive.rightArm.setPosition(0);

            offset = drive.imu.getAngularOrientation().firstAngle;
            drive.backs.setPosition(1);
            drive.turnSync(-offset, drive.RF, drive.LB);

            drive.frontYkA.setPosition(0.75);

            drive.setPoseEstimate(new Pose2d(45, -48, Math.toRadians(179)));

            drive.relayPose(telemetry, drive);

            //TODO: set new pose after empirical analysis

            //sleep(500000);

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            //.strafeRight(2)
                            .forward(32)
                            .build()
            );

        }


    }

}
