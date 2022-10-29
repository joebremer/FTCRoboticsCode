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

        claw.setPosition(1);
        this.sleep(500);
        driveForwardForIN(4);
        driveStrafeForIN(36.8*dir, 0.7f);
        driveForwardForIN(32);
        moveSlideToPosition(0.95f);
        driveForwardForIN(4,0.3f);
        claw.setPosition(clawRestingPos);
        this.sleep(500);
        driveForwardForIN(-4,0.3f);
        moveSlideToPosition(0f);
        driveForwardForIN(-30);
    }
}