package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.configs.Field_Locations;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;

@Autonomous
public class New_Red extends LinearOpMode {

    String SkystoneLocation;
    String grab = "Grab";
    String drop = "Drop";

    @Override
    public void runOpMode() throws InterruptedException{

        SampleMecanumDriveREVOptimized drive = new SampleMecanumDriveREVOptimized(hardwareMap, true);

        Field_Locations locations = new Field_Locations();

        ConstantInterpolator FACING_BZ = new ConstantInterpolator(Math.toRadians(0));

        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();

        SkystoneLocation = drive.detectSkystone();

        drive.setPoseEstimate(new Pose2d(-36, -65, Math.toRadians(0)));

        if(SkystoneLocation == "Left" || SkystoneLocation =="NOT FOUND") {

            drive.backYkA.setPosition(1);

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .lineTo(locations.redSkystone4V, FACING_BZ)
                            .build()
            );

            drive.setPoseEstimate(locations.redSkystone4P);

            //TODO: add grab code

            drive.backYk.setPosition(1);
            sleep(200);
            drive.backYkA.setPosition(0.5);

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .lineTo(locations.redPassingVector, FACING_BZ)
                            .lineTo(locations.redFoundationPlate1V, FACING_BZ)
                            .build()
            );

            //TODO: add drop code

            drive.moveGrab(drop, this);

            drive.setPoseEstimate(locations.redFoundationPlate1P);

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .lineTo(locations.redSkystone1V, FACING_BZ)
                            .build()
            );

            //TODO: add grab code
            drive.moveGrab(grab, this);

            drive.setPoseEstimate(locations.redSkystone1P);

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .lineTo(locations.redPassingVector, FACING_BZ)
                            .lineTo(locations.redFoundationPlate1V, FACING_BZ)
                            .build()
            );

            //TODO: add drop code
            drive.moveGrab(drop, this);

            drive.setPoseEstimate(locations.redFoundationPlate1P);

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .lineTo(locations.redPlateGrabVector, FACING_BZ)
                            .build()
            );

            drive.turnLeftGyro(Math.toRadians(90), this, telemetry);

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .back(5)
                            .build()
            );

            //TODO: clamp code

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .forward(15)
                            .build()
            );

            drive.turnLeftGyro(Math.toRadians(90), this, telemetry);

            //TODO: unclamp code

            drive.getPoseEstimate();
            
        }

    }

}
