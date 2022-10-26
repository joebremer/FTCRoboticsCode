package org.firstinspires.ftc.teamcode.automodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutoMode;

@Autonomous(name="AUTO, B, PLACE", group="B")
public class AutoMode_B_Place extends AutoMode {

    @Override
    protected void autoMode() throws InterruptedException {
        claw.setPosition(1);
        this.sleep(500);
        driveForwardForIN(4);
        driveStrafeForIN(36.8, 0.7f);
        driveForwardForIN(32);
        moveSlideToPosition(0.95f);
        driveForwardForIN(4,0.3f);
        claw.setPosition(clawRestingPos);
        this.sleep(500);
        driveForwardForIN(-4,0.3f);
        moveSlideToPosition(0f);
        driveForwardForIN(-32);
    }
}