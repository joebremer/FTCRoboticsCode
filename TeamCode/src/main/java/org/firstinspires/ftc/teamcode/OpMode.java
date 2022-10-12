package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

//joe add comments goteem
@TeleOp(name="OPMODE", group="Opmodes")
public class OpMode extends Controller {
    protected void operate(){


        drive();

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        imu.getPosition();
        telemetry.addData("angle: ",angles.firstAngle);
        telemetry.update();

        /*if(gamepad1.a){
            grabber.setPosition(1);
        } else {
            grabber.setPosition(0);
        }*/

        /*if(gamepad2.right_bumper || gamepad2.right_trigger > 0) {
            if(gamepad2.right_bumper) {
                intake.setPower(0.6);
            } else {
                intake.setPower(gamepad2.right_trigger);
            }
        } else if(gamepad2.left_trigger > 0.7){
            intake.setPower(-0.85);
        } else {
            intake.setPower(0);
        }

        if(gamepad2.dpad_up) {
            greenThingSpeed += greenThingAcceleration;
            greenThingMotion = 1;
        } else if(gamepad2.dpad_down){
            greenThingSpeed -= greenThingAcceleration;
            greenThingMotion = -1;
        } else {
            if(greenThingMotion != 0){
                greenThingSpeed = greenThingBrake*greenThingMotion*(-1);
                greenThingMotion = 0;
            }
           greenThingSpeed /= 1.2;
        }

        if(greenThingSpeed > greenThingSpeedMax){
            greenThingSpeed = greenThingSpeedMax;
        } else if (greenThingSpeed < -greenThingSpeedMax){
            greenThingSpeed = -greenThingSpeedMax;
        }

        greenThing.setPower(greenThingSpeed);

        if(gamepad1.left_bumper){
            speed = 0.16;
        } else {
            speed = 1.0;
        }

        if(gamepad2.dpad_left){
            amongusPos += amongusSpeed;
        } else if(gamepad2.dpad_right){
            amongusPos -= amongusSpeed;
        }

        if(gamepad2.y) {
            amongusPos = 0.914;
        }

        amongusPos = Math.max(Math.min(amongusPos, 0.955), 0.4);
        amongus.setPosition(amongusPos);

        telemetry.addData("Slide Pos", linearSlide.getCurrentPosition());
        telemetry.addData("wrist", wrist.getController().getServoPosition(0));
        telemetry.update();*/
    }

    @Override
    public void runOpMode() {
        setup();

        while (opModeIsActive()) {
           operate();
        }
    }
}
