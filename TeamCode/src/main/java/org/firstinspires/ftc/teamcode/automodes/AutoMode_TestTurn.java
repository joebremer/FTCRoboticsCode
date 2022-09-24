package org.firstinspires.ftc.teamcode.automodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.AutoMode;
import org.firstinspires.ftc.teamcode.Controller;

@Autonomous(name="Autonomous_TestTurn")
public class AutoMode_TestTurn extends AutoMode {

    @Override
    protected void autoMode() throws InterruptedException {
        driveTurnToHeading(0.5,90);
        //driveForwardForIN(2);
        //driveTurnToHeading(0.5,90);
    }
}