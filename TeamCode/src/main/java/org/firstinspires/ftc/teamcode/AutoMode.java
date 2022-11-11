package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@Autonomous(name="Autonomous")
public class AutoMode extends Controller{

    protected void autoMode() throws InterruptedException { }

    @Override
    public void runOpMode() {
        setup();
        try {
            autoMode();
            claw.setPosition(clawRestingPos);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    public void movePlaceCone(float mirror){
        float dir = Math.signum(mirror);

        claw.setPosition(clawClosedPos);
        this.sleep(900);
        moveSlideToPosition(0.1f);
        driveForwardForIN(4);
        driveStrafeForIN(32*dir, 0.7f);
        driveForwardForIN(32);
        moveSlideToPosition(slideGoalPositionHigh);
        driveForwardForIN(5,0.6f);
        claw.setPosition(clawRestingPos);
        this.sleep(500);

        /*driveForwardForIN(-5,0.6f);
        moveSlideToPosition(0f);
        driveStrafeForIN(-12*dir);
        driveForwardForIN(29.2);
        driveTurnForAngle(-90*dir);
        driveForwardForIN(46);


        for(int i = 0; i < 1; i++) {
            moveSlideToPosition(0.05f);
            driveForwardForIN(13.5,0.8);
            claw.setPosition(clawClosedPos);
            this.sleep(900);
            moveSlideToPosition(0.2f);
            driveForwardForIN(-8);
            moveSlideToPosition(0.1f);
            driveForwardForIN(-35);
            driveTurnForAngle(90*dir);
            moveSlideToPosition(slideGoalPositionHigh);
            driveForwardForIN(2.5, 0.6);
            claw.setPosition(clawRestingPos);
            this.sleep(900);
            driveForwardForIN(-2.5, 0.6);
            moveSlideToPosition(0f);
            driveTurnForAngle(-90*dir);
            driveForwardForIN(30);
        }

        driveStrafeForIN(-48);
        this.sleep(6000);*/

        driveForwardForIN(-5,0.3f);
        moveSlideToPosition(0f);
        driveForwardForIN(-25);
    }
}