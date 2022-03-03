package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.openCV.SkystoneDeterminationExample;
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
public class AutoNoRoadrunner extends LinearOpMode {
    OpenCvInternalCamera phoneCam;
    SkystoneDeterminationExample.SkystoneDeterminationPipeline pipeline;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Robot robot = new Robot(hardwareMap);

        //if we should strafe towards the wall
        SampleMecanumDrive.wall = true;

        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        long autoTime = System.currentTimeMillis();

        //red / blue side
        int side = 1;

        //if we should do 2nd pre load
        boolean preload2 = false;

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

        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);

        while (!isStarted())
        {

            if(gamepad1.x) side = -1;
            if(gamepad1.b) side = 1;

            if(gamepad1.dpad_up) preload2 = true;
            if(gamepad1.dpad_down) preload2 = false;

            telemetry.addLine("Analysis" + pipeline.getAnalysis());
            if(side == 1) {
                telemetry.addLine(String.format("<big><font color=#%02x%02x%02x>red</font><big>", 255, 0, 0));
                        //.addData("side", " red");
            }
            else {
                telemetry.addLine(String.format("<big><font color=#%02x%02x%02x>blue</font><big>", 0, 0, 255));
                        //.addData("side", " blue");
            }

            if(preload2) {
                telemetry.addLine(String.format("<big><font color=#%02x%02x%02x>2nd preload</font><big>", 0, 255, 0));
            }
            else {
                telemetry.addLine(String.format("<big><font color=#%02x%02x%02x>no 2nd preload</font><big>", 255, 0, 0));
            }

            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }



        if (opModeIsActive()) {

            drive.setPoseEstimate(new Pose2d(side == 1 ? -5 /*red*/ : 0 /*blue*/,-64,side == 1 ? 0 /*red*/ : 180 /*blue*/));

            autoTime = System.currentTimeMillis();

            int level = pipeline.getAnalysis().ordinal() + 1;

            phoneCam.closeCameraDevice();

            robot.setLevel(level);

            //put encoder servo down
            robot.encoderservo.setPosition(0.25);

            telemetry.addData("Location", level);

            robot.extendState = Robot.ExtendState.EXTEND;

            //intake down
            if(preload2){
                //reverse for 2nd preload
                robot.setIntakeBucketState(side == 1 ? Robot.IntakeBucket.LEFT /*red*/ : Robot.IntakeBucket.RIGHT /*blue*/);
            }
            else {
                robot.setIntakeBucketState(side == 1 ? Robot.IntakeBucket.RIGHT /*red*/ : Robot.IntakeBucket.LEFT /*blue*/);
            }

            //drive to hub
            drive.setDrivePower(new Pose2d(-0.5 * side,0,0));
            while (opModeIsActive() && robot.extendState != Robot.ExtendState.RESET) {
                drive.updatePoseEstimate();
                robot.updateExtend();
                robot.updateLiftServo();
                robot.updateIntakeBucket();
                if(drive.getPoseEstimate().getX() < -15){
                    drive.setDrivePower(new Pose2d(0,0,0));
                }
            }
            drive.setDrivePower(new Pose2d(0,0,0));

            robot.setIntakeSpeed(1);
            if(preload2){
                //2nd preload
                drive.setDrivePower(new Pose2d(-0.3 * side,0,0));
                while (robot.getColor(side * -1) > 2 && opModeIsActive() && System.currentTimeMillis() - autoTime < 4000) {
                    drive.updatePoseEstimate();
                    robot.updateExtend();
                    robot.updateLiftServo();
                    robot.updateIntakeBucket();
                }
                drive.setDrivePower(new Pose2d(0,0,0));

                robot.setIntakeBucketState(Robot.IntakeBucket.UP);
                robot.intakebucketClock = System.currentTimeMillis();

                while(System.currentTimeMillis() - robot.intakebucketClock < 1500) {
                    //for(int i = 0;i < 2000; i ++) {
                    robot.updateIntakeBucket();
                    //}
                }

                robot.extendState = Robot.ExtendState.EXTEND;

                //drive to hub
                drive.setDrivePower(new Pose2d(0.5 * side,0,0));
                while (opModeIsActive() && robot.extendState != Robot.ExtendState.RESET) {
                    drive.updatePoseEstimate();
                    robot.updateExtend();
                    robot.updateLiftServo();
                    robot.updateIntakeBucket();
                    if(drive.getPoseEstimate().getX() > -23){
                        drive.setDrivePower(new Pose2d(0,0,0));
                    }
                }
                drive.setDrivePower(new Pose2d(0,0,0));
            }

            while(opModeIsActive()) {

                robot.setIntakeBucketState(side == 1 ? Robot.IntakeBucket.RIGHT /*red*/ : Robot.IntakeBucket.LEFT /*blue*/);

                robot.setIntakeSpeed(1);

                //drive into warehouse
                drive.setDrivePower(new Pose2d(0.75 * side,0,0));
                while (robot.getColor(side) > 2 && opModeIsActive()) {
                    drive.updatePoseEstimate();
                    robot.updateExtend();
                    robot.updateLiftServo();
                    robot.updateIntakeBucket();
                    if(drive.getPoseEstimate().getX() > 0){
                        drive.setDrivePower(new Pose2d(0.3 * side,0,0));
                    }
                }

                drive.setDrivePower(new Pose2d(0,0,0));

                //turn off intake and raise intake bucket
                robot.setIntakeSpeed(-1);
                sleep(100);
                robot.stopIntake();
                robot.setIntakeBucketState(Robot.IntakeBucket.UP);

                telemetry.addData("Time: ", System.currentTimeMillis() - autoTime);
                telemetry.update();

                if(System.currentTimeMillis() - autoTime > 25000){
                    while(opModeIsActive())
                    {
                        telemetry.addData("Time: ", System.currentTimeMillis() - autoTime);
                        telemetry.addData("","Stopped");
                        telemetry.update();
                    }

                }

                //drive to hub
                drive.setDrivePower(new Pose2d(-0.75 * side,0,0));
                while (drive.getPoseEstimate().getX() > -10 && opModeIsActive()) {
                    drive.updatePoseEstimate();
                    robot.updateExtend();
                    robot.updateLiftServo();
                    robot.updateIntakeBucket();
                    robot.stopIntake();
                }
                drive.setDrivePower(new Pose2d(0,0,0));

                //set target to high goal and extend
                robot.setLevel(3);
                robot.extendState = Robot.ExtendState.EXTEND;

                //intake down
                robot.setIntakeBucketState(side == 1 ? Robot.IntakeBucket.RIGHT /*red*/ : Robot.IntakeBucket.LEFT /*blue*/);

                //wait until we finished dumping
                while (robot.extendState != Robot.ExtendState.RESET && opModeIsActive()) {
                    robot.updateExtend();
                    robot.updateLiftServo();
                    robot.stopIntake();
                }
            }
        }
    }
}