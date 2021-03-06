package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "drive")
public class ResetEncoders extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);

        waitForStart();

        if(!isStopRequested()) {
            robot.extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.extend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            robot.intakePivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.intakePivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }
}
