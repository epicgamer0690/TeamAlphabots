package org.firstinspires.ftc.teamcode.Autonomous_WORKING_ON;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Carousel_Test", group="Training")
public class Carousel_Test extends LinearOpMode {

    DcMotor carouselMotor;


    @Override
    public void runOpMode()  {
        carouselMotor = hardwareMap.get(DcMotor.class, "carousel_arm");
        waitForStart();
        carouselFunc();
    }

    public void sleep(int milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    public void carouselFunc(){
        carouselMotor.setPower(0.3);
        sleep(5000);
        carouselMotor.setPower(0);
    }
}
