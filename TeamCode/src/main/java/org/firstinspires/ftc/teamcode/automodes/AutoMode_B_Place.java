package org.firstinspires.ftc.teamcode.automodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutoMode;

@Autonomous(name="AUTO, B, PLACE", group="B")
public class AutoMode_B_Place extends AutoMode {

    @Override
    protected void autoMode() throws InterruptedException {
        movePlaceCone(-1);
    }
}