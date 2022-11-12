package org.firstinspires.ftc.teamcode;

import android.content.Context;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Controller extends LinearOpMode{
    protected DcMotor mtr_fl;
    protected DcMotor mtr_fr;
    protected DcMotor mtr_bl;
    protected DcMotor mtr_br;

    protected DcMotor slide;
    protected int slideStartPosition;
    protected int slideSize = -3200;
    protected int slidePad = -600;
    protected float slideSpeedUp = 0.8f;
    protected float slideSpeedDown = 0.5f;
    protected float slideSupportSpeed = 0.18f;

    protected float slideGoalPositionHigh = 0.85f;
    protected float slideGoalPositionMedium = 0.5f;


    protected Servo grabber;

    protected Servo claw;
    protected float clawRestingPos = 0f;
    protected float clawClosedPos = 0.8f;

    protected BNO055IMU imu;

    protected int mtr_fl_position = 0;
    protected int mtr_fr_position = 0;
    protected int mtr_bl_position = 0;
    protected int mtr_br_position = 0;
    protected int minMotorDist = 45;

    //protected DcMotor greenThing;
    //protected DcMotor linearSlide;
    //protected DcMotor intake;
    //protected Servo amongus;
    //protected Servo wrist;

    protected boolean gopmode_enabled = true;

    //protected float intakeDirection = 0f;
    //protected boolean intakeButtonPressed = false;

    //protected boolean inReverse = true;
    //protected boolean leftBumperPressed = false;

    protected double speed = 1.0;
    protected double turnSpeed = 1.0;

    //protected double greenThingSpeed = 0;
    //protected double greenThingSpeedMax = 1;
    //protected double greenThingAcceleration = 0.045;
    //protected int greenThingMotion = 0;
    //protected double greenThingBrake = 0.25;

    //protected double wristDefaultPos = 0.79;

    //protected double amongusPos = 0.95;
    //protected double amongusSpeed = 0.005;

    protected float wheelDiameter = 320; //mm
    protected float wheelFullRotation = 1440; //encoder increments

    protected int lowPos = 0;

    protected boolean inPresetMovement = false;

    //protected Context myApp;
    //protected SoundPlayer.PlaySoundParams params;
    //protected int soundID;

    protected void setup(){
        mtr_fl  = hardwareMap.get(DcMotor.class, "motor-fl");
        mtr_fr  = hardwareMap.get(DcMotor.class, "motor-fr");
        mtr_bl  = hardwareMap.get(DcMotor.class, "motor-bl");
        mtr_br  = hardwareMap.get(DcMotor.class, "motor-br");

        slide = hardwareMap.get(DcMotor.class, "slide");
        slideStartPosition = slide.getCurrentPosition();

        claw = hardwareMap.get(Servo.class, "claw");

        //grabber = hardwareMap.get(Servo.class, "claw");
        //grabber = hardwareMap.get(Servo.class, "claw");

        //greenThing = hardwareMap.get(DcMotor.class, "greenthing");
        //intake = hardwareMap.get(DcMotor.class, "intake");
        //linearSlide = hardwareMap.get(DcMotor.class, "linearslide");
        //amongus = hardwareMap.get(Servo.class, "amongus");
        //wrist = hardwareMap.get(Servo.class, "wrist");

        mtr_fl.setDirection(DcMotor.Direction.FORWARD);
        mtr_bl.setDirection(DcMotor.Direction.FORWARD);
        mtr_fr.setDirection(DcMotor.Direction.REVERSE);
        mtr_br.setDirection(DcMotor.Direction.REVERSE);

        mtr_fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mtr_bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mtr_fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mtr_br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        //lowPos = linearSlide.getCurrentPosition()+120;

        //myApp = hardwareMap.appContext;

        // create a sound parameter that holds the desired player parameters.
        //params = new SoundPlayer.PlaySoundParams();
        //params.loopControl = 0;
        //params.waitForNonLoopingSoundsToFinish = true;

        runWithEncoders(true);
        waitForStart();

        //wrist.setPosition(wristDefaultPos);
        sleep(10);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    protected void runWithEncoders(boolean isOn){
        if(isOn){
            mtr_fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            mtr_fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            mtr_bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            mtr_br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            mtr_fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            mtr_fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            mtr_bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            mtr_br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } else {
            mtr_fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            mtr_fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            mtr_bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            mtr_br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            mtr_fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            mtr_fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            mtr_bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            mtr_br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    protected double[] calculateMove(double forward, double strafe, double turn){
        double r = Math.hypot(strafe, forward); // hypo means hypotenuse
        double robotAngle = Math.atan2(forward, -strafe) - Math.PI / 4;
        double rightX = -turn*turnSpeed;
        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;
        return new double[] {v1,v2,v3,v4};
    }

    protected void startDrive(double forward, double strafe, double turn){
        double[] ctrl = calculateMove(forward, strafe, turn);
        mtr_fl.setPower(ctrl[0]);
        mtr_fr.setPower(ctrl[1]);
        mtr_bl.setPower(ctrl[2]);
        mtr_br.setPower(ctrl[3]);
    }

    protected void stopDrive(){
        mtr_fl.setPower(0);
        mtr_fr.setPower(0);
        mtr_bl.setPower(0);
        mtr_br.setPower(0);
    }

    protected void drive(){
        double ctrl_forward = 0;
        if(gamepad1.right_trigger != 0){
            ctrl_forward = -gamepad1.right_trigger*1.5;
            if(gamepad1.left_trigger > 0.8){
                ctrl_forward = -gamepad1.right_trigger/4;
            }
        } else {
            ctrl_forward = gamepad1.left_trigger*2;
        }
        double ctrl_strafe = gamepad1.right_stick_x;
        double ctrl_turn = gamepad1.left_stick_x;

        double[] ctrl = calculateMove(ctrl_forward, ctrl_strafe, ctrl_turn);

        mtr_fl.setPower(ctrl[0] * speed);
        mtr_fr.setPower(ctrl[1] * speed);
        mtr_bl.setPower(ctrl[2] * speed);
        mtr_br.setPower(ctrl[3] * speed);

        mtr_fl_position = mtr_fl.getCurrentPosition();
        mtr_fr_position = mtr_fr.getCurrentPosition();
        mtr_bl_position = mtr_bl.getCurrentPosition();
        mtr_br_position = mtr_br.getCurrentPosition();
    }

    protected boolean numDistLessThan(int num_a, int num_b){
        if(Math.abs(num_a-num_b) < minMotorDist){
            return true;
        } else {
            return false;
        }
    }

    protected boolean isDriveBusy(){
        if(numDistLessThan(mtr_fl.getCurrentPosition(), mtr_fl.getTargetPosition())  && numDistLessThan(mtr_fr.getCurrentPosition(), mtr_fr.getTargetPosition()) && numDistLessThan(mtr_bl.getCurrentPosition(), mtr_bl.getTargetPosition()) && numDistLessThan(mtr_br.getCurrentPosition(), mtr_br.getTargetPosition())){
            return false;
        } else {
            return true;
        }
    }

    protected void driveForwardForMM(double fDist, double spd){
        stopDrive();
        runWithEncoders(true);

        //mm
        double fNumRotations = -fDist/(Math.PI*wheelDiameter);
        int targetPos = (int) (fNumRotations*wheelFullRotation);

        mtr_fl.setTargetPosition(targetPos);
        mtr_fr.setTargetPosition(targetPos);
        mtr_bl.setTargetPosition(targetPos);
        mtr_br.setTargetPosition(targetPos);

        mtr_fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mtr_fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mtr_bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mtr_br.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        startDrive(spd,0,0);

        while(isDriveBusy() && opModeIsActive()){
            sleep(100);
        }
    }

    protected void driveForwardForIN(double fDist, double spd){ driveForwardForMM(fDist*25.4, spd); }
    protected void driveForwardForIN(double fDist){
        driveForwardForIN(fDist, 1);
    }

    protected void driveStrafeForMM(double fDist, double spd){
        stopDrive();
        runWithEncoders(true);

        //mm
        int targetPos = (int) (fDist*(8.5/Math.PI));

        double ctrl[] = calculateMove(0,1,0);

        mtr_fl.setTargetPosition((int) (targetPos*ctrl[0]));
        mtr_fr.setTargetPosition((int) (targetPos*ctrl[1]));
        mtr_bl.setTargetPosition((int) (targetPos*ctrl[2]));
        mtr_br.setTargetPosition((int) (targetPos*ctrl[3]));

        mtr_fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mtr_fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mtr_bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mtr_br.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        startDrive(0,spd,0);

        while(isDriveBusy() && opModeIsActive()){
            sleep(100);
        }
    }

    protected void driveStrafeForIN(double fDist, double spd){
        driveStrafeForMM(fDist*25.4, spd);
    }
    protected void driveStrafeForIN(double fDist){
        driveStrafeForIN(fDist, 1);
    }

    protected void driveTurnForAngle(double aTurn){
        stopDrive();
        runWithEncoders(true);

        double spd = 0.5;

        //mm
        //int targetPos = (int) (aTurn*11.775);
        int targetPos = (int) (aTurn*9.3);

        double ctrl[] = calculateMove(0,0,1);

        mtr_fl.setTargetPosition((int) (targetPos*ctrl[0]));
        mtr_fr.setTargetPosition((int) (targetPos*ctrl[1]));
        mtr_bl.setTargetPosition((int) (targetPos*ctrl[2]));
        mtr_br.setTargetPosition((int) (targetPos*ctrl[3]));

        mtr_fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mtr_fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mtr_bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mtr_br.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        startDrive(0,0,spd);

        while(isDriveBusy() && opModeIsActive()){
            sleep(100);
        }
    }

    protected float slidePercentageToEncoderPosition(float percentPosition){
        return slideStartPosition+slideSize*Math.max(0,Math.min(1,percentPosition));
    }

    protected void moveSlideToPosition(float percentPosition){
        float slideCurrentPosition = slidePercentageToEncoderPosition(percentPosition);

        if(slide.getCurrentPosition() > slideCurrentPosition){
            while(slide.getCurrentPosition() > slideCurrentPosition && opModeIsActive()){
                slide.setPower(-slideSpeedUp);
                this.sleep(100);
            }
        } else {
            while(slide.getCurrentPosition() < slideCurrentPosition && opModeIsActive()){
                slide.setPower(slideSpeedDown);
                this.sleep(100);
            }
        }

        slide.setPower(-slideSupportSpeed);
    }

    @Override
    public void runOpMode() {}

    protected void autoMode() throws InterruptedException {};
}
