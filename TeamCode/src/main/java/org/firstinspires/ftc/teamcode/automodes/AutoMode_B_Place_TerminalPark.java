package org.firstinspires.ftc.teamcode.automodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutoMode;

@Autonomous(name="AUTO, B, PLACE, TERMINAL PARK", group="B")
public class AutoMode_B_Place_TerminalPark extends AutoMode {

    @Override
    protected void autoMode() throws InterruptedException {
        movePlaceCone(-1,true);
    }
}