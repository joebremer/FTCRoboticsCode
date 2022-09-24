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

    protected Servo grabber;

    protected BNO055IMU imu;

    protected double robotHeading  = 0;
    protected double headingOffset = 0;
    protected double headingError  = 0;
    private double targetHeading = 0;

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     COUNTS_PER_MOTOR_REV    = 537.7 ;   // eg: GoBILDA 312 RPM Yellow Jacket
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    // These constants define the desired driving/control characteristics
    // They can/should be tweaked to suit the specific robot drive train.
    static final double     DRIVE_SPEED             = 0.4;     // Max driving speed for better distance accuracy.
    static final double     TURN_SPEED              = 0.2;     // Max Turn speed to limit turn rate
    static final double     HEADING_THRESHOLD       = 1.0 ;    // How close must the heading get to the target before moving to next step.
    // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.
    // Define the Proportional control coefficient (or GAIN) for "heading control".
    // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
    // Increase these numbers if the heading does not corrects strongly enough (eg: a heavy robot or using tracks)
    // Decrease these numbers if the heading does not settle on the correct value (eg: very agile robot with omni wheels)
    static final double     P_TURN_GAIN            = 0.005;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_GAIN           = 0.03;     // Larger is more responsive, but also less stable

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

    protected boolean gopmode_enabled = false;

    //protected float intakeDirection = 0f;
    //protected boolean intakeButtonPressed = false;

    protected boolean inReverse = true;
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

        grabber = hardwareMap.get(Servo.class, "claw");

        //greenThing = hardwareMap.get(DcMotor.class, "greenthing");
        //intake = hardwareMap.get(DcMotor.class, "intake");
        //linearSlide = hardwareMap.get(DcMotor.class, "linearslide");
        //amongus = hardwareMap.get(Servo.class, "amongus");
        //wrist = hardwareMap.get(Servo.class, "wrist");
        mtr_fl.setDirection(DcMotor.Direction.FORWARD);
        mtr_bl.setDirection(DcMotor.Direction.FORWARD);
        mtr_fr.setDirection(DcMotor.Direction.REVERSE);
        mtr_br.setDirection(DcMotor.Direction.REVERSE);

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
            ctrl_forward = gamepad1.right_trigger*1.5;
            if(gamepad1.left_trigger > 0.8){
                ctrl_forward = gamepad1.right_trigger/4;
            }
        } else {
            ctrl_forward = -gamepad1.left_trigger*2;
        }
        double ctrl_strafe = -gamepad1.right_stick_x;
        double ctrl_turn = gamepad1.left_stick_x;

        if(gamepad1.b) {
            mtr_fl.setTargetPosition(mtr_fl_position);
            mtr_fr.setTargetPosition(mtr_fr_position);
            mtr_bl.setTargetPosition(mtr_bl_position);
            mtr_br.setTargetPosition(mtr_br_position);

            mtr_fl.setPower(0.25);
            mtr_fr.setPower(0.25);
            mtr_bl.setPower(0.25);
            mtr_br.setPower(0.25);

            mtr_fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            mtr_fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            mtr_bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            mtr_br.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else {
            if(mtr_fl.getMode() == DcMotor.RunMode.RUN_TO_POSITION) {
                runWithEncoders(true);
            }

            if (gamepad1.dpad_up) {
                ctrl_forward = -1;
            } else if (gamepad1.dpad_down) {
                ctrl_forward = 1;
            }

            if (inReverse) {
                ctrl_forward *= -1;
                ctrl_strafe *= -1;
            }
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

        while(isDriveBusy()){
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
        int targetPos = (int) (fDist*(9/Math.PI));

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

        while(isDriveBusy()){
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

        double spd = 0.75;

        //mm
        //int targetPos = (int) (aTurn*11.775);
        int targetPos = (int) (aTurn*9.375);

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

        while(isDriveBusy()){
            sleep(100);
        }
    }

    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param maxTurnSpeed Desired MAX speed of turn. (range 0 to +1.0)
     * @param heading Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *              0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *              If a relative angle is required, add/subtract from current heading.
     */
    public void driveTurnToHeading(double maxTurnSpeed, double heading) {

        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(heading, P_DRIVE_GAIN);

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {
            startDrive(0,0,Range.clip(getSteeringCorrection(heading, P_TURN_GAIN), -maxTurnSpeed, maxTurnSpeed));
        }

        // Stop all motion;
        stopDrive();
    }

    /**
     * This method uses a Proportional Controller to determine how much steering correction is required.
     *
     * @param desiredHeading        The desired absolute heading (relative to last heading reset)
     * @param proportionalGain      Gain factor applied to heading error to obtain turning power.
     * @return                      Turning power needed to get to required heading.
     */
    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;  // Save for telemetry

        // Get the robot heading by applying an offset to the IMU heading
        robotHeading = getRawHeading() - headingOffset;

        // Determine the heading current error
        headingError = targetHeading - robotHeading;

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180)  headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    /**
     * read the raw (un-offset Gyro heading) directly from the IMU
     */
    public double getRawHeading() {
        Orientation angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    /**
     * Reset the "offset" heading back to zero
     */
    public void resetHeading() {
        // Save a new heading offset equal to the current raw heading.
        headingOffset = getRawHeading();
        robotHeading = 0;
    }

    @Override
    public void runOpMode() {}

    protected void autoMode() throws InterruptedException {};
}
