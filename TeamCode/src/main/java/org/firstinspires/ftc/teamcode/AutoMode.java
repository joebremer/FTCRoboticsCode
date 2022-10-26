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
}