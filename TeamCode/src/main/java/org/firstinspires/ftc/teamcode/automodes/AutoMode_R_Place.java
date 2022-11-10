package org.firstinspires.ftc.teamcode.automodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutoMode;

@Autonomous(name="AUTO, R, PLACE", group="R")
public class AutoMode_R_Place extends AutoMode {

    @Override
    protected void autoMode() throws InterruptedException {
        movePlaceCone(1);
    }
}