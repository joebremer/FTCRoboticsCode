package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="SLIDE TEST", group="Opmodes")
public class SlideTest extends Controller {
    protected void operate(){

        float slidePower = 0;

        if(gamepad2.dpad_up){
            if(slide.getCurrentPosition() < slideStartPosition) {
                slidePower = slideSpeedUp;
            }
            slide.setPower(slidePower);
        } else if(gamepad2.dpad_down) {
            if(slide.getCurrentPosition() > slideStartPosition+slideSize) {
                slidePower = -slideSpeedDown;
            }
            slide.setPower(slidePower);
        } else {
            slide.setPower(0);
        }



        telemetry.addData("SlideSpeed", slide.getPower());
        telemetry.addData("SlidePosition", slide.getCurrentPosition()-slideStartPosition);
        telemetry.update();
    }

    @Override
    public void runOpMode() {
        setup();

        while (opModeIsActive()) {
           operate();
        }
    }
}
