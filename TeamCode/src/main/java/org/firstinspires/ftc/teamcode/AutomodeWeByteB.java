package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@Autonomous(name="AutomodeWeByte")
public class AutomodeWeByteB extends Controller {
    protected void autoMode() throws InterruptedException {
        driveStrafeForIN(-48);
    }
}