package org.firstinspires.ftc.teamcode.Autonomous_WORKING_ON;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Carousel_Test", group="Training")
public class Carousel_Test extends OpMode {

    DcMotor carouselMotor;

    @Override
    public void init() {
        carouselMotor = hardwareMap.get(DcMotor.class, "carousel_arm");



    }
    @Override
    public void start()  {

    }
    public void loop(){

    }
    public void sleep(int milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    public void carouselFunc() throws InterruptedException{
        carouselMotor.setPower(0.5);
        sleep(5000);
        carouselMotor.setPower(0);
    }
}
