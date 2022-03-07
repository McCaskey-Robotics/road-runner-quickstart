package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "drive")
public class DriverControled extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Robot robot = new Robot(hardwareMap);

        //if we should strafe towards the wall
        SampleMecanumDrive.wall = false;

        double bucketPos = 0.08;
        double liftPos = 0.93;
        double intakePos = 0.45;

        boolean lastGuide = false;

        boolean erik = false;
        boolean dpadDownLast = false;

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        //put encoder servo down
        robot.encoderservo.setPosition(0);

        robot.autoDump = false;

        while (!isStopRequested()) {

            //Erik
            if (gamepad1.x)
            {
                erik = true;
            }

            if (gamepad1.y)
            {
                erik = false;
            }

            if(erik) {
                drive.setWeightedDrivePower(
                        new Pose2d(
                                gamepad1.left_stick_x / 1.5,
                                -gamepad1.left_stick_y / 1.5,
                                -gamepad1.right_stick_y / 2
                        )
                );
            }
            //Renee
            else{
                drive.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.left_stick_y / 1.5,
                                -gamepad1.left_stick_x / 1.5,
                                -gamepad1.right_stick_x / 2
                        )
                );
            }

            boolean g = gamepad2.guide;

            //shared
            if(gamepad2.square){

                robot.autoExtend =  !g;

                robot.setTarget("shared");
                robot.extendState = Robot.ExtendState.EXTEND;
            }

            //high goal
            if(gamepad2.triangle){

                robot.autoExtend =  !g;

                robot.setTarget("high");
                robot.extendState = Robot.ExtendState.EXTEND;
            }

            //cap
            if(gamepad2.cross){
                robot.autoExtend =  !g;

                robot.setTarget("cap");
                robot.extendState = Robot.ExtendState.EXTEND;
            }

            //cancle extend
            if(gamepad2.left_trigger > 0.5)
                robot.extendState = Robot.ExtendState.RESET;


            if(gamepad2.left_bumper){
                robot.extendState = Robot.ExtendState.DUMP;
            }

            if(Math.abs(gamepad2.left_stick_y) > 0.1 || Math.abs(gamepad2.right_stick_y) > 0.1) {
                if(robot.extendState != Robot.ExtendState.MANUAL) {
                    robot.l = robot.lift1.getPosition();
                    robot.extendState = Robot.ExtendState.MANUAL;
                }
            }

            robot.updateExtend(gamepad2);
            robot.updateLiftServo();

            /* INTAKE BUCKET */
            /*if(gamepad2.dpad_up){
                robot.setIntakeBucketState(Robot.IntakeBucket.UP);
            }
            else if(gamepad2.dpad_right){
                robot.setIntakeBucketState(Robot.IntakeBucket.RIGHT);
            }
            else if(gamepad2.dpad_left){
                robot.setIntakeBucketState(Robot.IntakeBucket.LEFT);
            }

            boolean d = gamepad2.dpad_down;

            if(d && !dpadDownLast){
                robot.setIntakeBucketState(robot.intakeBucketlastState);
            }
            dpadDownLast = d;*/

            //robot.updateIntakeBucket();


             if(gamepad2.dpad_left){
                robot.setIntakeBucketState(Robot.IntakeBucket.LEFT);
            }
            else if(gamepad2.dpad_right){
                robot.setIntakeBucketState(Robot.IntakeBucket.RIGHT);
            }
            else if(gamepad2.dpad_up){
                robot.setIntakeBucketState(Robot.IntakeBucket.UP);
            }

            robot.updateIntakeBucket();



            if (!gamepad2.right_bumper ^ (robot.intakePivot.getCurrentPosition() > 0)) {
                robot.setIntakeSpeed(-gamepad2.right_trigger);
            } else {
                robot.setIntakeSpeed(gamepad2.right_trigger);
            }


            /* CAROUSEL */
            robot.setCarSpeed(gamepad1.right_trigger - gamepad1.left_trigger);



            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();

            //position
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());

            //motor encoders
            telemetry.addData("extend: ", robot.extend.getCurrentPosition());
            telemetry.addData("lift1: ", robot.lift1.getPosition());
            telemetry.addData("lift2: ", robot.lift2.getPosition());
            telemetry.addData("bucket: ", robot.bucket.getPosition());
            telemetry.addData("intake: ", intakePos);

            telemetry.addData("encoderServo: ", robot.encoderservo.getPosition());

            //digital channels
            telemetry.addData("auto extend: ", robot.autoExtend);
            telemetry.addData("auto dump: ", robot.autoDump);
            telemetry.addData("extendState: ", robot.extendState);

            //color sensor
            telemetry.addData("intake Pivot ", robot.intakePivot.getCurrentPosition());
            telemetry.addData("intake s ", robot.intakeBucketState);
            telemetry.addData("intake sin ", Math.sin(Math.toRadians((robot.intakePivot.getCurrentPosition() / 130.0) * 90.0)));
            telemetry.addData("intake deg ", (robot.intakePivot.getCurrentPosition() / 130.0) * 90.0);

            //telemetry.addData("alpha ", robot.getColor(0,1));
            //telemetry.addData("red ", robot.getColor(1,1));
            //telemetry.addData("green ", robot.getColor(2,1));
            //telemetry.addData("blue ", robot.getColor(3,1));

            //telemetry.addData("distance ", robot.getColor(-1,2));
            //telemetry.addData("alpha ", robot.getColor(0,2));
            //telemetry.addData("red ", robot.getColor(1,2));
            //telemetry.addData("green ", robot.getColor(2,2));
            //telemetry.addData("blue ", robot.getColor(3,2));

            telemetry.addData("time: ", System.currentTimeMillis() - robot.intakeClock);
            telemetry.addData("time: ", robot.intakeState);
            telemetry.update();
        }
    }
}