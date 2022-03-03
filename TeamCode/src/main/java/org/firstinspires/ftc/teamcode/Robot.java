package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

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
    public Servo bucket;

    public Servo encoderservo;

    public Servo tapeliftservo;
    public CRServo taperotateservo;
    public CRServo tapeextendservo;

    public static double tapelift = 0.34;
    public double liftPosition = 0.4;
    public double tapeRotateSpeed = 0;

    public double sharedExtend = 700;

    public static double carMaxSpeed = 0.43;

    public DigitalChannel extendStop;

    public static double bucketLevelMultiplier = 0.6;
    public static double bucketOffset1 = 0.1;
    public static double bucketOffset2 = 0.025;

    public static int extendOffset = -200;

    int level = 3;

    //if we should wait after we have extended
    public boolean autoDump = true;

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
        INIT,       //initialize extend
        MANUAL,     //manual control
        RESET,      //spin intake
        EXTEND,     //lift for transfer
        WAITTODUMP, //wiat to dump
        DUMP,       //reset
        WAIT        //wait for dump
    }

    enum IntakeBucket {
        LEFT,       //left
        UP,     //up
        RIGHT,      //right
    }

    enum driver2 {
        tape,       //left
        other,     //up
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
        bucket = hardwareMap.get(Servo.class, "bucket");
        intakePivot = hardwareMap.get(DcMotor.class, "intakePivot");

        encoderservo = hardwareMap.get(Servo.class, "encoderservo");

        tapeextendservo = hardwareMap.get(CRServo.class, "tapeextendservo");
        tapeliftservo = hardwareMap.get(Servo.class, "tapeliftservo");
        taperotateservo = hardwareMap.get(CRServo.class, "taperotateservo");

        extendStop = hardwareMap.get(DigitalChannel.class,"extendStop");

        extend.setDirection(DcMotorSimple.Direction.FORWARD);
        extend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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
        if(this.intakeBucketState != intakeBucketState) {

            intakeBucketlastState = this.intakeBucketState;

            this.intakeBucketState = intakeBucketState;

            intakebucketClock = System.currentTimeMillis();
        }
    }

    public void updateIntakeBucket(){
        /*if(intakeBucketState == IntakeBucket.UP){
            if(intakeBucketlastState == IntakeBucket.RIGHT){
                intake.setPosition(0.5);
            }
            else if(intakeBucketlastState == IntakeBucket.LEFT) {
                intake.setPosition(0.3);
            }
        }
        else if(intakeBucketState == IntakeBucket.RIGHT){
            intake.setPosition(0.12);
        }
        else if(intakeBucketState == IntakeBucket.LEFT) {
            intake.setPosition(0.72);
        }*/
    }

    public void updateExtend() throws InterruptedException {
        switch (extendState){
            //manual
            case MANUAL:{
                extend.setPower(0);
            }
            //init
            case INIT:{

                extendState = ExtendState.RESET;

            }
            //reset
            case RESET:{

                if(extend.getCurrentPosition() > 125){
                    extend.setTargetPosition(0);
                    extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    extend.setPower(1);
                }

                if(level > 0) {
                    if (extend.getCurrentPosition() < 1700) {
                        bucket.setPosition(0);
                        setLiftPosition(0.04);
                    }
                }
                else{
                    if (extend.getCurrentPosition() < 100) {
                        if(System.currentTimeMillis() - extendClock2 > 1500) {
                            bucket.setPosition(0);
                            extendClock2 = System.currentTimeMillis();
                        }

                        if(System.currentTimeMillis() - extendClock2 > 500)
                            setLiftPosition(0.04);
                    }
                }

                if(!extend.isBusy()){
                    extend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    extend.setPower(0);
                }

                extendClock = System.currentTimeMillis();
                break;
            }
            //extend + lift bucket
            case EXTEND:{

                if(bucket.getPosition() > 0.4) {
                    bucket.setPosition((liftPosition * bucketLevelMultiplier) + bucketOffset2);
                }
                else{
                    bucket.setPosition((liftPosition * bucketLevelMultiplier) + bucketOffset1);
                }


                //shared
                if (level == 0)
                {
                    if(System.currentTimeMillis() - extendClock > 500) {
                        extend.setTargetPosition((int)sharedExtend);

                        extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        extend.setPower(1);

                        extendClock = System.currentTimeMillis();
                        extendState = ExtendState.WAITTODUMP;
                    }

                    setLiftPosition(0.35);
                }

                else if (level == 1)
                {
                    extend.setTargetPosition(2200 + extendOffset);
                    setLiftPosition(0.22);
                }
                else if (level == 2)
                {
                    extend.setTargetPosition(2100 + extendOffset);
                    setLiftPosition(0.47);
                }
                else
                {
                    extend.setTargetPosition(2800 + extendOffset);
                    setLiftPosition(0.73);
                }

                if(level != 0) {
                    extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    extend.setPower(1);

                    extendClock = System.currentTimeMillis();
                    extendState = ExtendState.WAITTODUMP;
                }

                break;
            }
            //wait to dump (gives me a head start on the extend)
            case WAITTODUMP:{

                levelBucket();

                if(autoDump){
                    extendState = ExtendState.DUMP;
                }
            }
            //dump
            case DUMP:{

                levelBucket();

                if (System.currentTimeMillis() - extendClock > 400 && !extend.isBusy()) {
                    if (level == 0) {
                        bucket.setPosition(0.85 + bucketOffset1);
                    }
                    if (level == 1) {
                        bucket.setPosition(0.6 + bucketOffset1);
                    } else if (level == 2) {
                        bucket.setPosition(0.75 + bucketOffset1);
                    } else {
                        bucket.setPosition(0.93 + bucketOffset1);
                    }

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

    void levelBucket(){
        if(bucket.getPosition() > 0.2) {
            bucket.setPosition((liftPosition * bucketLevelMultiplier) + bucketOffset2);
        }
        else{
            bucket.setPosition((liftPosition * bucketLevelMultiplier) + bucketOffset1);
        }
    }

    public void setLevel(int level) {
        this.level = level;
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
        if(Math.abs(lift1.getPosition() - liftPosition) > 0.1) {
            if (lift1.getPosition() > liftPosition) {
                lift1.setPosition(lift1.getPosition() - 0.1);
            } else if (lift1.getPosition() < liftPosition) {
                lift1.setPosition(lift1.getPosition() + 0.1);
            }
        }
        else{
            lift1.setPosition(liftPosition);
        }
    }

    //void setExtendSpeed(double s){
        //if(!extend.isBusy()) {
        //    if (s < 0 && extendStop.getState()) s = 0;
        //    extend.setPower(s);
        //}
    //}

    void setExtendPos(int p){
        extend.setTargetPosition(p);
        extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extend.setPower(1);
        while(extend.isBusy()){

        }
        extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
            return sensorDistance1.getDistance(DistanceUnit.INCH);
        }
        return sensorDistance2.getDistance(DistanceUnit.INCH);
    }
}
