package org.firstinspires.ftc.teamcode.automodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.AutoMode;

@Disabled
@Autonomous(name="AUTOMODE SLIDETEST", group="B")
public class SlideTestAutoMode extends AutoMode {

    //@Override
    protected void slideTes() throws InterruptedException {
        claw.setPosition(1);
        this.sleep(500);
        moveSlideToPosition(0.95f);
        claw.setPosition(clawRestingPos);
        this.sleep(500);
        moveSlideToPosition(0);
    }
}