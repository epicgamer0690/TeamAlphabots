package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.lang.Thread;

@Autonomous(name="TestingDriveOpMode", group="Training")
    public class TestingDriveOpMode extends OpMode {



    DcMotor leftWheel;
    DcMotor rightWheel;
    DcMotor backLeftWheel;
    DcMotor backRightWheel;
    //DcMotor rightWheel;
    double drivePower = 0.5;

    @Override
    public void init() {
        leftWheel = hardwareMap.dcMotor.get("left_wheel");
        rightWheel = hardwareMap.dcMotor.get("right_wheel");
        backRightWheel = hardwareMap.dcMotor.get("back_right_wheel");
        backLeftWheel = hardwareMap.dcMotor.get("back_left_wheel");


    }
    @Override
    public void loop()   {

        setForwardDirection();
        setForwardPower(0.5);
        sleep(5000);
        setForwardPower(0);


    }



    //@Override
    //public void loop() {
    //    leftWheel.setPower(drivePower);
    //    rightWheel.setPower(drivePower);
    //    backRightWheel.setPower(drivePower);
    //    backLeftWheel.setPower(drivePower);


//    }


    public void diagonalLeft() {
        backLeftWheel.setDirection(DcMotor.Direction.REVERSE);
        rightWheel.setDirection(DcMotor.Direction.FORWARD);

        rightWheel.setPower(drivePower);
        backLeftWheel.setPower(drivePower);
        //sleep(milliseconds 1000);

    }

    public void backwardsDiagonalLeft() {
        backLeftWheel.setDirection(DcMotor.Direction.FORWARD);
        rightWheel.setDirection(DcMotor.Direction.REVERSE);

        rightWheel.setPower(drivePower);
        backLeftWheel.setPower(drivePower);
        //sleep(2000);

    }

    public void diagonalRight() {
        backRightWheel.setDirection(DcMotor.Direction.FORWARD);
        leftWheel.setDirection(DcMotor.Direction.REVERSE);

        leftWheel.setPower(drivePower);
        backRightWheel.setPower(drivePower);
        //ElapsedTime(200);
    }

    public void horizontalRight() {
        backRightWheel.setDirection(DcMotor.Direction.FORWARD);
        leftWheel.setDirection(DcMotor.Direction.REVERSE);
        rightWheel.setDirection(DcMotor.Direction.REVERSE);
        backLeftWheel.setDirection(DcMotor.Direction.FORWARD);

        leftWheel.setPower(drivePower);
        backRightWheel.setPower(drivePower);
        rightWheel.setPower(drivePower);
        backLeftWheel.setPower(drivePower);
        //sleep(2000);
    }

    public void horizontalLeft() {
        backRightWheel.setDirection(DcMotor.Direction.REVERSE);
        leftWheel.setDirection(DcMotor.Direction.FORWARD);
        rightWheel.setDirection(DcMotor.Direction.FORWARD);
        backLeftWheel.setDirection(DcMotor.Direction.REVERSE);

        leftWheel.setPower(drivePower);
        backRightWheel.setPower(drivePower);
        rightWheel.setPower(drivePower);
        backLeftWheel.setPower(drivePower);
    }

    public void backwardsDiagonalRight() {
        backRightWheel.setDirection(DcMotor.Direction.REVERSE);
        leftWheel.setDirection(DcMotor.Direction.FORWARD);

        leftWheel.setPower(drivePower);
        backRightWheel.setPower(drivePower);

    }

    public void setForwardDirection() {

        leftWheel.setDirection(DcMotor.Direction.REVERSE);
        rightWheel.setDirection(DcMotor.Direction.FORWARD);
        backLeftWheel.setDirection(DcMotor.Direction.REVERSE);
        backRightWheel.setDirection(DcMotor.Direction.FORWARD);

    }

    public void setForwardPower(double drivePower);

        leftWheel.setPower(drivePower);
        rightWheel.setPower(drivePower);
        backRightWheel.setPower(drivePower);
        backLeftWheel.setPower(drivePower);


    }

    public void backward() {

        leftWheel.setDirection(DcMotor.Direction.FORWARD);
        rightWheel.setDirection(DcMotor.Direction.REVERSE);
        backLeftWheel.setDirection(DcMotor.Direction.FORWARD);
        backRightWheel.setDirection(DcMotor.Direction.REVERSE);

        leftWheel.setPower(drivePower);
        rightWheel.setPower(drivePower);
        backRightWheel.setPower(drivePower);
        backLeftWheel.setPower(drivePower);


    }





}