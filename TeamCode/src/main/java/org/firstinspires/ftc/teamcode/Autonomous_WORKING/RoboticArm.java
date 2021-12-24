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
    double intergralSum = 0;
    double Kp, Ki, Kd = 0;
    private double lastError;

    private ElapsedTime timer = new ElapsedTime();

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
        /*
        shippingHubLevel(65);
        resetEncoders();

        shippingHubLevel(125);
        resetEncoders();

        shippingHubLevel(195);
        resetEncoders();

         */

        shippingHubLevelPID(3);

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

    public void sleep(int milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }


    public void shippingHubLevel(int rotation) {
        armMotor.setTargetPosition(rotation);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(1);
        Sleep(1000);
        armMotor.setTargetPosition(-rotation);
        armMotor.setPower(0.04);
    }
    public double PIDControl(double reference, double state){
        double error = reference - state;
        intergralSum += error * timer.seconds();
        double derivative = (error - lastError) * timer.seconds();
        lastError = error;

        timer.reset();

        double output = (error * Kp) + (derivative * Kd) + (intergralSum * Ki);
        return(output);
    }
    public void shippingHubLevelPID(int shippingLevel) {
        resetEncoders();
        int reference = 0;
        switch (shippingLevel) {
            case 0:
                armMotor.setTargetPosition(10);
                reference = 10;
                break;
            case 1:
                armMotor.setTargetPosition(90);
                reference = 90;
                break;
            case 2:
                armMotor.setTargetPosition(120);
                reference = 120;
                break;
            case 3:
                armMotor.setTargetPosition(150);
                reference = 150;
                break;
        }
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while(Math.abs(reference - armMotor.getCurrentPosition()) > 5){
            armMotor.setPower(PIDControl(reference, armMotor.getCurrentPosition()));
        }
    }
}