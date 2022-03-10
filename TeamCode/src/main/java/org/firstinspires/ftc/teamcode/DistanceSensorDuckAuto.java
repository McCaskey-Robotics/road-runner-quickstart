package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.openCV.SkystoneDeterminationExample;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@Autonomous()
public class DistanceSensorDuckAuto extends LinearOpMode
{
    OpenCvInternalCamera phoneCam;
    SkystoneDeterminationExample.SkystoneDeterminationPipeline pipeline;

    @Override
    public void runOpMode() throws InterruptedException
    {

        int side = 1;

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Robot robot = new Robot(hardwareMap);

        robot.autoExtend = true;
        robot.autoDump = true;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        pipeline = new SkystoneDeterminationExample.SkystoneDeterminationPipeline();
        phoneCam.setPipeline(pipeline);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.MAXIMIZE_EFFICIENCY);
        phoneCam.showFpsMeterOnViewport(false);

        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                phoneCam.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });


        robot.encoderservo.setPosition(0.25);

        int level = pipeline.getAnalysis().ordinal() + 1;

        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);

        while (!isStarted())
        {
            //choose side
            if (gamepad1.b)
            {
                side = 1;
                drive.setPoseEstimate(new Pose2d(-42.5, -64, 0));
            }
            if (gamepad1.x)
            {
                side = -1;
                drive.setPoseEstimate(new Pose2d(-42.5, 64, Math.toRadians(180)));
            }

            if (side == 1)
            {
                telemetry.addLine(String.format("<big><font color=#%02x%02x%02x>Red</font><big>", 255, 0, 0));
            }
            else
            {
                telemetry.addLine(String.format("<big><font color=#%02x%02x%02x>Blue</font><big>", 0, 0, 255));
            }

            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
            level = pipeline.getAnalysis().ordinal() + 1;


        }

        phoneCam.stopStreaming();

        if (opModeIsActive())
        {

            robot.setLevel(level);

            telemetry.addData("Location", level);

            ElapsedTime runtime = new ElapsedTime();

            robot.setLiftPosition(0.17);

            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 1.0)) {
                robot.updateLiftServo();
            }


            //strafe 10 in away from wall
            drive.setDrivePower(new Pose2d(0,0.5,0));
            double d = robot.getDistanceBack();
            while((d < 10 || d > 50 || Double.isNaN(d))  && opModeIsActive()){
                d = robot.getDistanceBack();
            }
            drive.setDrivePower(new Pose2d(0, 0, 0));

            //drive to carousel
            if(side == 1) {//red
                d = robot.getDistanceLeft();
                drive.setDrivePower(new Pose2d(-0.5 * side, 0, 0));
                while ((d > 8 || Double.isNaN(d)) && opModeIsActive()) {
                    d = robot.getDistanceLeft();
                }
                drive.setDrivePower(new Pose2d(0, -0.5, 0));
                sleep(500);
                drive.setDrivePower(new Pose2d(0, 0, 0));
            }
            else{//blue
                drive.turn(Math.toRadians(90));
                while (robot.getDistanceBack() > 2 && opModeIsActive()) {
                    drive.setDrivePower(new Pose2d(0, -0.5, 0));
                }


                drive.setDrivePower(new Pose2d(-0.5, 0, 0));
                sleep(250);
                drive.setDrivePower(new Pose2d(0, 0, 0));

            }

            //spin duck
            robot.setCarSpeed(-0.5 * side);
            sleep(4000);
            robot.setCarSpeed(0);

            if(side == 1) {//red
                //drive away from carousel and turn
                drive.setDrivePower(new Pose2d(0.5, 0.5, 0));
                sleep(500);
                drive.setDrivePower(new Pose2d(0, 0, 0));

                drive.setDrivePower(new Pose2d(0, 0, -0.5));
                drive.updatePoseEstimate();
                while(Math.toDegrees(drive.getPoseEstimate().getHeading()) > 280 || Math.toDegrees(drive.getPoseEstimate().getHeading()) < 90){
                    drive.updatePoseEstimate();
                    telemetry.addData("heading" , Math.toDegrees(drive.getPoseEstimate().getHeading()));
                    telemetry.update();
                }
                drive.setDrivePower(new Pose2d(0, 0, 0));

                //drive to wall
                drive.setDrivePower(new Pose2d(0, -0.5, 0));
                sleep(2000);
                drive.setDrivePower(new Pose2d(0, 0, 0));
            }

            if(side == 1) { //red
                //drive over to place preloaded
                while ((robot.getDistanceRight() < 30 || robot.getDistanceRight() > 50) && opModeIsActive()) {
                    drive.setDrivePower(new Pose2d(-0.5, 0, 0));
                }
            }
            else{ //blue
                //drive over to place preloaded
                while (robot.getDistanceLeft() < 30 && opModeIsActive()) {
                    drive.setDrivePower(new Pose2d(0.5, 0, 0));
                }
            }
            drive.setDrivePower(new Pose2d(0, 0, 0));

            //strafe away from wall
            while (robot.getDistanceBack() < 11 && opModeIsActive()) {
                drive.setDrivePower(new Pose2d(0, 0.5, 0));
            }
            drive.setDrivePower(new Pose2d(0, 0, 0));

            //place freight
            robot.extendState = Robot.ExtendState.EXTEND;
            runtime.reset();
            while(robot.extendState != Robot.ExtendState.WAITTOEXTEND && runtime.seconds() < 5.0 && opModeIsActive()){
                robot.updateExtend();
                robot.updateLiftServo();
            }

            //park

            //drive to wall
            drive.setDrivePower(new Pose2d(0, -0.5, 0));
            sleep(1000);
            drive.setDrivePower(new Pose2d(0, 0, 0));

            if(side == 1) { //red
                while (robot.getDistanceRight() > 26 && opModeIsActive()) {
                    drive.setDrivePower(new Pose2d(0.5, 0, 0));
                }
            }
            else{ //blue
                while (robot.getDistanceLeft() > 26 && opModeIsActive()) {
                    drive.setDrivePower(new Pose2d(-0.5, 0, 0));
                }
            }
            drive.setDrivePower(new Pose2d(0, 0, 0));

        }
    }
}
