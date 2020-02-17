package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;
import org.firstinspires.ftc.teamcode.util.MecanumPowers;
import org.firstinspires.ftc.teamcode.util.MecanumUtil;
import org.openftc.revextensions2.ExpansionHubEx;

@TeleOp(name="Teleop Facing Left")
public class DriverOpRD extends LinearOpMode {
    SampleMecanumDriveREVOptimized robot;

    boolean headingAdjust = true;

    public static final double OUTSIDE = 0.75;
    public static final double INSIDE = 0.01;

    @Override
    public void runOpMode(){

        robot = new SampleMecanumDriveREVOptimized(hardwareMap, false);
        //robot.detector.webcam.stopStreaming();

        boolean heartbeat = false;
        boolean strobe = false;
        double voltage;

        ElapsedTime time = new ElapsedTime();

        robot.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE);

        double startingAngle = robot.getExternalHeading();

        //robot.primeServo();
        telemetry.addLine("Ready");
        //Pepega Clap 777 Oi 3Head ANY BRUVS?

        waitForStart();

        robot.turner.setPosition(INSIDE);
        robot.backYk.setPosition(0);
        robot.backYkA.setPosition(0.1);
        robot.frontYk.setPosition(0.5);
        robot.frontYkA.setPosition(0.75);

        time.reset();
        time.startTimeNanoseconds();
        telemetry.clearAll();

        while(opModeIsActive()){

            //telemetry.addData("Time", time.time());
            //telemetry.update();

            voltage = robot.hub.read12vMonitor(ExpansionHubEx.VoltageUnits.VOLTS);
            if(voltage < 12.55){
                telemetry.addData("Voltage", voltage);
                telemetry.addLine("Voltage has reached critically low levels");
                telemetry.update();
            }
            if(voltage >= 12.55){
                telemetry.addData("Voltage", voltage);
                telemetry.addLine("Voltage is okay");
                telemetry.update();
            }



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
                double leftX = gamepad1.left_stick_y; //-gamepad1.left_stick_x;
                double leftY = -gamepad1.left_stick_x;
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

            telemetry.addData("RF enc", robot.RF.getCurrentPosition());
            telemetry.addData("RB enc", robot.RB.getCurrentPosition());
            telemetry.addData("LF enc", robot.LF.getCurrentPosition());
            telemetry.addData("LB enc", robot.LB.getCurrentPosition());
            telemetry.addData("left odo", robot.lColl.getCurrentPosition());
            telemetry.addData("right odo", robot.rColl.getCurrentPosition());
            telemetry.addData("back odo", robot.spool2.getCurrentPosition());

            if(gamepad1.start){
                headingAdjust = false;
            }
            if(gamepad1.back){
                headingAdjust = true;
            }

            double spoolpowr = gamepad2.right_trigger - gamepad2.left_trigger;
            robot.spool.setPower(spoolpowr);
            robot.spool2.setPower(spoolpowr);

            if(gamepad1.a){
                robot.backs.setPosition(0);
            }
            if(gamepad1.b){
                robot.backs.setPosition(1);
            }

            if(gamepad1.y){

                robot.leftArm.setPosition(0);
                robot.rightArm.setPosition(1);

            }

            if(gamepad1.x){

                robot.leftArm.setPosition(1);
                robot.rightArm.setPosition(0);

            }

            if(gamepad1.right_bumper){
                robot.lColl.setPower(-0.55);
                robot.rColl.setPower(0.55);
            }

            if(gamepad1.left_bumper){
                robot.lColl.setPower(0);
                robot.rColl.setPower(0);
            }

            if(gamepad1.left_trigger > 0){
                robot.lColl.setPower(0.3);
                robot.rColl.setPower(-0.3);
            }

            if(gamepad1.dpad_left){
                robot.followTrajectorySync(
                        robot.trajectoryBuilder()
                                .strafeLeft(1.5)
                                .build()
                );
            }

            if(gamepad1.dpad_right){
                robot.followTrajectorySync(
                        robot.trajectoryBuilder()
                                .strafeRight(1.5)
                                .build()
                );
            }

            if(gamepad1.dpad_up){
                robot.followTrajectorySync(
                        robot.trajectoryBuilder()
                                .forward(1.5)
                                .build()
                );
            }

            if(gamepad1.dpad_down){
                robot.followTrajectorySync(
                        robot.trajectoryBuilder()
                                .back(1.5)
                                .build()
                );

            }

            if (gamepad2.a){

                robot.grabber.setPosition(1);

            }

            if(gamepad2.b){

                robot.grabber.setPosition(0);

            }

            if(gamepad2.dpad_left){
                robot.turner.setPosition(OUTSIDE);
            }

            if(gamepad2.dpad_right){
                robot.turner.setPosition(INSIDE);
            }

            if(gamepad2.left_bumper){
                robot.cap.setPosition(0.55);
            }

            if (gamepad2.right_bumper){
                robot.cap.setPosition(1);
            }

            if(gamepad2.left_stick_y < 0){


                while(gamepad2.left_stick_y < 0) {
                    robot.LF.setPower(-gamepad2.left_stick_y*-0.2);
                    robot.LB.setPower(-gamepad2.left_stick_y*0.2);
                    robot.RF.setPower(-gamepad2.left_stick_y*0.2);
                    robot.RB.setPower(-gamepad2.left_stick_y*-0.2);
                }
                robot.killAll();
            }

            if(gamepad2.left_stick_y > 0){

                while(gamepad2.left_stick_y > 0) {
                    robot.LF.setPower(gamepad2.left_stick_y*0.2);
                    robot.LB.setPower(gamepad2.left_stick_y*-0.2);
                    robot.RF.setPower(gamepad2.left_stick_y*-0.2);
                    robot.RB.setPower(gamepad2.left_stick_y*0.2);
                }
                robot.killAll();

            }


        }

    }

}


