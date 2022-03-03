package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.openCV.SkystoneDeterminationExample;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.Vector;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@Autonomous()
public class DuckAuto extends LinearOpMode
{
    OpenCvInternalCamera phoneCam;
    SkystoneDeterminationExample.SkystoneDeterminationPipeline pipeline;

    @Override
    public void runOpMode() throws InterruptedException
    {
        double retractTimer;

        boolean red = true;

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Robot robot = new Robot(hardwareMap);

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

        //Red trajectories
        //start to carousel
        TrajectorySequence seqR1 = drive.trajectorySequenceBuilder(new Pose2d(-42.5, -64, 0))
                .lineTo(new Vector2d(-42.5, -48))
                .waitSeconds(0.2)
                .lineTo(new Vector2d(-61, -48))
                .waitSeconds(0.2)
                .lineTo(new Vector2d(-61.5, -60))
                .build();

        //carousel to hub
        TrajectorySequence seqR2 = drive.trajectorySequenceBuilder(seqR1.end())
                .lineTo(new Vector2d(-55, -42))
                .waitSeconds(0.2)
                .turn(Math.toRadians(-90))
                .waitSeconds(0.5)
                .build();

        TrajectorySequence seqR3 = drive.trajectorySequenceBuilder(new Pose2d(-76, -42, Math.toRadians(-90)))
                .lineTo(new Vector2d(-76, -36))
                .waitSeconds(0.2)
                .lineTo(new Vector2d(-56, -36))
                .build();

        //hub to parking
        TrajectorySequence seqR4 = drive.trajectorySequenceBuilder(seqR3.end())
                .lineTo(new Vector2d(-76, -33))
                .waitSeconds(0.2)
                .lineTo(new Vector2d(-76, -48))
                .build();




        //Blue Trajectories
        //start to carousel
        TrajectorySequence seqB1 = drive.trajectorySequenceBuilder(new Pose2d(-42.5,64,Math.toRadians(180)))
                .lineTo(new Vector2d(-42.5, 40))
                .waitSeconds(0.2)
                .turn(Math.toRadians(90))
                .waitSeconds(0.2)
                .build();

        //carousel to hub
        TrajectorySequence seqB2 = drive.trajectorySequenceBuilder(seqB1.end())
                .lineTo(new Vector2d(-55, 48))
                .waitSeconds(0.2)
                .lineTo(new Vector2d(-74, 48))
                .waitSeconds(0.2)
                .lineTo(new Vector2d(-74, 36))
                .waitSeconds(0.2)
                .lineTo(new Vector2d(-54, 36))
                .build();

        //hub to parking
        TrajectorySequence seqB3 = drive.trajectorySequenceBuilder(seqB2.end())
                .lineTo(new Vector2d(-76, 33))
                .waitSeconds(0.2)
                .lineTo(new Vector2d(-76, 50))
                .build();


        robot.encoderservo.setPosition(0.25);

        int level = pipeline.getAnalysis().ordinal() + 1;

        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);

        while (!isStarted())
        {
            //choose side
            if (gamepad1.b)
            {
                red = true;
                drive.setPoseEstimate(new Pose2d(-42.5, -64, 0));
            }
            if (gamepad1.x)
            {
                red = false;
                drive.setPoseEstimate(new Pose2d(-42.5, 64, Math.toRadians(180)));
            }

            if (red)
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

            robot.extendState = Robot.ExtendState.EXTEND;

            //intake down
            robot.setIntakeBucketState(Robot.IntakeBucket.RIGHT);

            //drive to carousel
            if (red)
            {
                drive.followTrajectorySequence(seqR1);
                drive.update();
                robot.setCarSpeed(-0.5);
                sleep(3000);
                robot.setCarSpeed(0);
            }
            else
            {
                drive.followTrajectorySequence(seqB1);
                drive.update();
                drive.setDrivePower(new Pose2d(0,-0.75,0));
                sleep(1500);
                drive.setDrivePower(new Pose2d(-0.2, 0, 0));
                sleep(1000);
                drive.setDrivePower(new Pose2d(0,0,0));

                robot.setCarSpeed(0.5);
                sleep(3000);
                robot.setCarSpeed(0);
            }

            //Turn so back is pressed against wall + drive to hub


            if (red)
            {
                drive.followTrajectorySequence(seqR2);
                drive.update();
                drive.setDrivePower(new Pose2d(0,-0.75,0));
                sleep(1150);

                drive.followTrajectorySequence(seqR3);
                drive.update();
            }

            //extend arm + deliver freight
            while (opModeIsActive() && robot.extendState != Robot.ExtendState.RESET)
            {
                robot.updateExtend();
                robot.updateLiftServo();
                robot.updateIntakeBucket();
            }


            //park + retract ar
            if (red)
            {
                drive.followTrajectorySequenceAsync(seqR4);
            }
            else 
            {
                drive.followTrajectorySequenceAsync(seqB3);
            }
            

            retractTimer = getRuntime();
            while (opModeIsActive() && (getRuntime() - retractTimer < 5))
            {
                drive.update();
                robot.updateExtend();
                robot.updateLiftServo();
            }
        }
    }
}
