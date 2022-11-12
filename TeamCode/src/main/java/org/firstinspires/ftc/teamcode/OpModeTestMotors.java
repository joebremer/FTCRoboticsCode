package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

//joe add comments goteem
@TeleOp(name="MOTORTEST", group="Opmodes")
public class OpModeTestMotors extends Controller {
    protected void operate(){
        if(gamepad1.a){
            mtr_fl.setPower(1);
        } else {
            mtr_fl.setPower(0);
        }
        if(gamepad1.b){
            mtr_fr.setPower(1);
        } else {
            mtr_fr.setPower(0);
        }
        if(gamepad1.x){
            mtr_bl.setPower(1);
        } else {
            mtr_bl.setPower(0);
        }
        if(gamepad1.y){
            mtr_br.setPower(1);
        } else {
            mtr_br.setPower(0);
        }
    }

    @Override
    public void runOpMode() {
        setup();

        while (opModeIsActive()) {
           operate();
        }
    }
}
