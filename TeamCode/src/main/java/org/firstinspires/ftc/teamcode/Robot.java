package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/*
 * Simple mecanum drive hardware implementation for REV hardware.
 */
@Config
public class Robot {

    public DcMotor extend;

    //non linear slide side
    public DcMotor intake;

    public DcMotor car;

    public DcMotor intakePivot;
    public Servo lift1;
    public Servo lift2;
    public ServoImplEx bucket;

    public Servo encoderservo;

    public double liftPosition = 0.155;

    public static double carMaxSpeed = 0.43;

    public DigitalChannel extendStop;

    public static double bucketLevelMultiplier = 0.6;
    public static double bucketOffset1 = 0.1;
    public static double bucketOffset2 = 0.025;

    public static double intakePivotPower = -0.4;
    public static double liftOffset = 0.001;

    public static double liftIncrement = 0.025;

    //if we should wait after we have extended
    public boolean autoDump = false;
    public boolean autoExtend = true;

    public double l = 0.5;

    public int extendTarget = 0;
    public double liftTarget = 0.155;

    public ColorSensor sensorColor1;
    public DistanceSensor sensorDistance1;

    public ColorSensor sensorColor2;
    public DistanceSensor sensorDistance2;

    public IntakeBucket getIntakeBucketState() {
        return intakeBucketState;
    }

    enum IntakeState {
        MANUAL,     //manual control
        INTAKE,     //spin intake
        LIFT,       //lift for transfer
        RESET       //reset
    }
    enum ExtendState {
        INIT,           //initialize extend
        MANUAL,         //manual control
        RESET,          //spin intake
        WAITTOEXTEND,   //Wait to Extend
        EXTEND,         //lift for transfer
        WAITTODUMP,     //wait to dump
        DUMP,           //reset
        WAIT            //wait for dump
    }

    enum IntakeBucket {
        LEFT,       //left
        UP,     //up
        RIGHT,      //right
    }

    public IntakeState intakeState = IntakeState.MANUAL;
    public ExtendState extendState = ExtendState.INIT;
    public IntakeBucket intakeBucketState = IntakeBucket.UP;
    public IntakeBucket intakeBucketlastState = IntakeBucket.UP;

    public long intakeClock = System.currentTimeMillis();
    public long extendClock = System.currentTimeMillis();
    public long extendClock2 = System.currentTimeMillis();
    public long intakebucketClock = System.currentTimeMillis();

    public int intake1Target = 0;
    public int intake2Target = 0;

    public boolean intake1IsVerticle = true;

    public Robot(HardwareMap hardwareMap) {
        extend = hardwareMap.get(DcMotor.class, "extend");
        intake = hardwareMap.get(DcMotor.class, "intake");
        car = hardwareMap.get(DcMotor.class, "car");

        lift1 = hardwareMap.get(Servo.class, "lift1");
        lift2 = hardwareMap.get(Servo.class, "lift2");
        bucket = hardwareMap.get(ServoImplEx.class, "bucket");
        intakePivot = hardwareMap.get(DcMotor.class, "intakePivot");

        encoderservo = hardwareMap.get(Servo.class, "encoderservo");

        extendStop = hardwareMap.get(DigitalChannel.class,"extendStop");

        extend.setDirection(DcMotorSimple.Direction.FORWARD);
        extend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        intakePivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakePivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        bucket.setPwmRange(new PwmControl.PwmRange(500, 2500));
        lift1.setDirection(Servo.Direction.REVERSE);
        lift2.setDirection(Servo.Direction.REVERSE);

        extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // get a reference to the color sensor.
        sensorColor1 = hardwareMap.get(ColorSensor .class, "color1");

        // get a reference to the distance sensor that shares the same name.
        sensorDistance1 = hardwareMap.get(DistanceSensor .class, "color1");

        // get a reference to the color sensor.
        sensorColor2 = hardwareMap.get(ColorSensor .class, "color2");

        // get a reference to the distance sensor that shares the same name.
        sensorDistance2 = hardwareMap.get(DistanceSensor .class, "color2");

    }

    public void setIntakeBucketState(IntakeBucket intakeBucketState) {
        this.intakeBucketState = intakeBucketState;
        /*if(this.intakeBucketState != intakeBucketState) {

            intakeBucketlastState = this.intakeBucketState;

            this.intakeBucketState = intakeBucketState;

            intakebucketClock = System.currentTimeMillis();
        }*/
    }

    public void updateIntakeBucket(){
        double p = 0;

        p = Math.sin(Math.toRadians((intakePivot.getCurrentPosition() / 130.0) * 90.0)) * Robot.intakePivotPower;
        if(intakeBucketState == IntakeBucket.UP){
            if(intakePivot.getCurrentPosition() > 10){
                p -= 0.25;
            }
            if(intakePivot.getCurrentPosition() < -10){
                p += 0.25;
            }
        }
        else if(intakeBucketState == IntakeBucket.LEFT){
            if(intakePivot.getCurrentPosition() < 125){
                p -= 0.35;
            }
        }
        else if(intakeBucketState == IntakeBucket.RIGHT) {
            if(intakePivot.getCurrentPosition() > -125){
                p += 0.35;
            }
        }

        intakePivot.setPower(p);
    }

    public void updateExtend() throws InterruptedException {
        updateExtend(new Gamepad());
    }

    public void updateExtend(Gamepad gamepad2) throws InterruptedException {
        switch (extendState){
            //manual
            case MANUAL:{
                l -= gamepad2.left_stick_y / 100;
                l = Math.min(0.8,l);
                l = Math.max(0.17,l);
                setLiftPosition(l);

                levelBucket();

                extend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                double y = gamepad2.right_stick_y;
                if(extend.getCurrentPosition() > 0 && y >= 0)
                    extend.setPower(-gamepad2.right_stick_y);
                else if(extend.getCurrentPosition() < 2500 && y <= 0)
                    extend.setPower(-gamepad2.right_stick_y);
                else{
                    extend.setPower(0);
                }
                break;
            }
            //init
            case INIT:{

                //extendState = ExtendState.RESET;

            }
            //reset
            case RESET:{

                if(extend.getCurrentPosition() > 125){
                    extend.setTargetPosition(0);
                    extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    extend.setPower(1);
                }

                //if(level > 0) {
                if (extend.getCurrentPosition() < 500) {
                    levelBucket();
                    setLiftPosition(0.17);
                }
                /*}
                else{
                    if (extend.getCurrentPosition() < 100) {
                        if(System.currentTimeMillis() - extendClock2 > 1500) {
                            levelBucket();
                            extendClock2 = System.currentTimeMillis();
                        }

                        if(System.currentTimeMillis() - extendClock2 > 500)
                            setLiftPosition(0.135);
                    }
                }*/

                if(!extend.isBusy()){
                    extend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    extend.setPower(0);
                    if(lift1.getPosition() <= 0.17 )
                        extendState = ExtendState.WAITTOEXTEND;
                }

                extendClock = System.currentTimeMillis();
                break;
            }
            case WAITTOEXTEND:{

                break;
            }
            //extend + lift bucket
            case EXTEND: {

                levelBucket();
                l = liftPosition;

                //shared
                /*if (level == 0)
                {
                    if(System.currentTimeMillis() - extendClock > 500) {
                        extend.setTargetPosition(700);

                        extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        extend.setPower(1);

                        extendClock = System.currentTimeMillis();
                        extendState = ExtendState.WAITTODUMP;
                    }

                    setLiftPosition(0.35);
                }

                if (level == 1)
                {
                    if(autoExtend)
                        extend.setTargetPosition(1200);
                    setLiftPosition(0.2);
                }
                else if (level == 2)
                {
                    if(autoExtend)
                        extend.setTargetPosition(1100);
                    setLiftPosition(0.4);
                }
                else
                {
                    if(autoExtend)
                        extend.setTargetPosition(1700);
                    setLiftPosition(0.65);
                }*/

                if (autoExtend){
                    extend.setTargetPosition(extendTarget);
                    extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    extend.setPower(1);
                }
                setLiftPosition(liftTarget);

                //if(level != 0) {
                extendClock = System.currentTimeMillis();
                extendState = ExtendState.WAITTODUMP;
                //}

                break;
            }
            //wait to dump (gives me a head start on the extend)
            case WAITTODUMP:{
                levelBucket();

                if(autoDump){
                    extendClock = System.currentTimeMillis();
                    extendState = ExtendState.DUMP;
                }
                else if(!extend.isBusy()){
                    extendState = ExtendState.MANUAL;
                }
                break;
            }
            //dump
            case DUMP:{

                levelBucket();


                if ((System.currentTimeMillis() - extendClock > 500 && !extend.isBusy()) || gamepad2.left_bumper) {
                    /*if (level == 0) {
                        bucket.setPosition(0.85 + bucketOffset1);
                    }
                    if (level == 1) {
                        bucket.setPosition(0.6 + bucketOffset1);
                    } else if (level == 2) {
                        bucket.setPosition(0.75 + bucketOffset1);
                    } else {
                        bucket.setPosition(0.825);
                    }*/

                    levelBucket(true);

                    extendClock = System.currentTimeMillis();
                    extendState = ExtendState.WAIT;
                }

                break;
            }
            case WAIT:{
                //wait 1/2 second to give the bucket time to dump then reset
                if(System.currentTimeMillis() - extendClock > 750) {
                    extendClock = System.currentTimeMillis();
                    extendState = ExtendState.RESET;
                }
            }
        }
    }

    void levelBucket(boolean d){
        if(d){
            double slope = (0.4 - 0.05) / (0.6 - 0.16);
            double b = (slope * (lift1.getPosition() - 0.16)) + 0.05;

            b += 0.4;

            if(Math.abs(b - bucket.getPosition()) > 0.01) {
                bucket.setPosition(b);
            }
        }
        else{
            levelBucket();
        }
    }

    void levelBucket(){
        //https://www.desmos.com/calculator/neehsxdyea

        double slope = (0.4 - 0.05) / (0.6 - 0.16);
        double b = (slope * (lift1.getPosition() - 0.16)) + 0.05;

        if(Math.abs(b - bucket.getPosition()) > 0.01) {
            bucket.setPosition(b);
        }
    }

    public void setTarget(String s){
        if(s.equals("low")){setTarget(1650,0.3);};
        if(s.equals("mid")){setTarget(1300,0.5);};
        if(s.equals("high")){setTarget(1700,0.65);};

        if(s.equals("shared")){setTarget(0,0.4);}

        //grab cap
        if(s.equals("cap")) setTarget(1500,0.35);
    }

    public void setTarget(int extendTarget, double liftTarget){
        this.extendTarget = extendTarget;
        this.liftTarget = liftTarget;
    }

    public void setLevel(int level) {
        if(level == 1)
            setTarget("low");
        if(level == 2)
            setTarget("mid");
        if(level == 3)
            setTarget("high");
    }

    public void setIntakeSpeed(double speed){
        if(true){
            intake.setPower(0 - speed);
        }
        else{
            intake.setPower(speed);
        }
    }

    public void stopIntake(){
        intake.setPower(0);
    }

    public void setLiftPosition(double p){
        liftPosition = p;
    }

    public void updateLiftServo(){
        if(Math.abs(lift1.getPosition() - liftPosition) > 0.03) {
            if (lift1.getPosition() > liftPosition) {
                lift1.setPosition(lift1.getPosition() - 0.03);
                lift2.setPosition(1 - (lift1.getPosition() + liftOffset));
            } else if (lift1.getPosition() < liftPosition) {
                lift1.setPosition(lift1.getPosition() + 0.024);
                lift2.setPosition(1 - (lift1.getPosition() + liftOffset));
            }
        }
        else{

        lift1.setPosition(liftPosition);
        lift2.setPosition(1 - (liftPosition + liftOffset));

        }
    }

    /**
     * @param s speed min: 0 Max: 1
     */
    void setCarSpeed(double s){
        car.setPower(s * carMaxSpeed);
    }


    // get the color sensor values
    double getColor(int s) {
        if(s == 1) {
            return sensorDistance2.getDistance(DistanceUnit.INCH);
        }
        return sensorDistance1.getDistance(DistanceUnit.INCH);
    }
}
