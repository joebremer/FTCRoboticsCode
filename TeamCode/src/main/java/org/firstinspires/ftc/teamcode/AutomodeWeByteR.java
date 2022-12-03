package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name="AutomodeWeByteR")
public class AutomodeWeByteR extends Controller {
    protected void autoMode() throws InterruptedException {
        driveStrafeForIN(48);
    }
}