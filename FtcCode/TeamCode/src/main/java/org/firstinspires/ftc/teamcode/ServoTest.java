package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="ServoTest", group="Training")
    public class ServoTest extends OpMode {



    DcMotor leftWheel;
    DcMotor rightWheel;
    DcMotor backLeftWheel;
    DcMotor backRightWheel;
    Servo servo;
    double drivePower = 0.5;
     //1 rotation = 360





    private ElapsedTime runtime= new ElapsedTime();

    @Override
    public void init() {
        leftWheel = hardwareMap.dcMotor.get("left_wheel");
        rightWheel = hardwareMap.dcMotor.get("right_wheel");
        backRightWheel = hardwareMap.dcMotor.get("back_right_wheel");
        backLeftWheel = hardwareMap.dcMotor.get("back_left_wheel");
        servo = hardwareMap.servo.get("front_servo");
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
        servo.setPosition(360);
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




    public void diagonalLeft(int rotation) {
        /*
        backLeftWheel.setDirection(DcMotor.Direction.REVERSE);
        rightWheel.setDirection(DcMotor.Direction.FORWARD);

        rightWheel.setPower(drivePower);
        backLeftWheel.setPower(drivePower);
        //sleep(milliseconds 1000);

         */

        rightWheel.setTargetPosition(rotation);
        backLeftWheel.setTargetPosition(-rotation);

        rightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightWheel.setPower(drivePower);
        backLeftWheel.setPower(-drivePower);



    }

    public void backwardsDiagonalLeft(int rotation) {
        /*
        backLeftWheel.setDirection(DcMotor.Direction.FORWARD);
        rightWheel.setDirection(DcMotor.Direction.REVERSE);

        rightWheel.setPower(drivePower);
        backLeftWheel.setPower(drivePower);
        //sleep(2000);

         */

        rightWheel.setTargetPosition(-rotation);
        backLeftWheel.setTargetPosition(rotation);

        rightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightWheel.setPower(-drivePower);
        backLeftWheel.setPower(drivePower);

    }

    public void diagonalRight(int rotation) {
        /*
        backRightWheel.setDirection(DcMotor.Direction.FORWARD);
        leftWheel.setDirection(DcMotor.Direction.REVERSE);

        leftWheel.setPower(drivePower);
        backRightWheel.setPower(drivePower);

         */

        leftWheel.setTargetPosition(-rotation);
        backRightWheel.setTargetPosition(rotation);

        leftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftWheel.setPower(-drivePower);
        backRightWheel.setPower(drivePower);
    }

    public void horizontalRight(int rotation) {

        leftWheel.setTargetPosition(-rotation);
        rightWheel.setTargetPosition(-rotation);
        backLeftWheel.setTargetPosition(rotation);
        backRightWheel.setTargetPosition(rotation);

        leftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftWheel.setPower(-drivePower);
        rightWheel.setPower(-drivePower);
        backLeftWheel.setPower(drivePower);
        backRightWheel.setPower(drivePower);
    }

    public void horizontalLeft(int rotation) {

        leftWheel.setTargetPosition(rotation);
        rightWheel.setTargetPosition(rotation);
        backLeftWheel.setTargetPosition(-rotation);
        backRightWheel.setTargetPosition(-rotation);

        leftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftWheel.setPower(drivePower);
        rightWheel.setPower(drivePower);
        backLeftWheel.setPower(-drivePower);
        backRightWheel.setPower(-drivePower);
    }

    public void backwardsDiagonalRight(int rotation) {

        leftWheel.setTargetPosition(rotation);
        backRightWheel.setTargetPosition(-rotation);

        leftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftWheel.setPower(drivePower);
        backRightWheel.setPower(-drivePower);

    }



    public void forward(int rotation) {


        leftWheel.setTargetPosition(-rotation);
        rightWheel.setTargetPosition(rotation);
        backLeftWheel.setTargetPosition(-rotation);
        backRightWheel.setTargetPosition(rotation);

        leftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftWheel.setPower(-drivePower);
        rightWheel.setPower(drivePower);
        backLeftWheel.setPower(-drivePower);
        backRightWheel.setPower(drivePower);




    }

    public void backward(int rotation) {


        leftWheel.setTargetPosition(rotation);
        rightWheel.setTargetPosition(-rotation);
        backLeftWheel.setTargetPosition(rotation);
        backRightWheel.setTargetPosition(-rotation);

        leftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftWheel.setPower(drivePower);
        rightWheel.setPower(-drivePower);
        backLeftWheel.setPower(drivePower);
        backRightWheel.setPower(-drivePower);



    }






}