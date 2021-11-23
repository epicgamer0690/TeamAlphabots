package org.firstinspires.ftc.teamcode.Autonomous_WORKING;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


import javax.tools.ForwardingFileObject;

@Autonomous(name="RoboticArm", group="Training")
public class RoboticArm extends OpMode {



    DcMotor leftWheel;
    DcMotor rightWheel;
    DcMotor backLeftWheel;
    DcMotor backRightWheel;
    DcMotor armMotor;
    DcMotor carouselMotor;
    double drivePower = 0.5;
    int rotation = 1000; //1 rotation = 360






    private ElapsedTime runtime= new ElapsedTime();

    public void carouselFunc() {

    }

    @Override
    public void init() {
        armMotor = hardwareMap.get(DcMotor.class, "expansion_motor");



    }
    public void Sleep(int milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
    public void resetEncoders() {
        leftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void start() {

        shippingHubLevel(65);
        resetEncoders();

        shippingHubLevel(125);
        resetEncoders();

        shippingHubLevel(195);
        resetEncoders();

        shippingHubLevel(10);
        resetEncoders();

    }

    @Override
    public void loop() {

    }
    @Override
    public void stop() {
        leftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


    }


    public void shippingHubLevel(int rotation) {
        armMotor.setTargetPosition(rotation);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(1);
        Sleep(1000);
        armMotor.setTargetPosition(-rotation);
        armMotor.setPower(0.04);
    }
}