package org.firstinspires.ftc.teamcode.Movement_Sensor_Test.sensor_test.movement_without_encoders;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Forwards", group="Training")
    public class Forwards extends OpMode {


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
    public void loop()  {
        forward();


    }

    public void forward() {

        leftWheel.setDirection(DcMotor.Direction.REVERSE);
        rightWheel.setDirection(DcMotor.Direction.FORWARD);
        backLeftWheel.setDirection(DcMotor.Direction.REVERSE);
        backRightWheel.setDirection(DcMotor.Direction.FORWARD);

        leftWheel.setPower(drivePower);
        rightWheel.setPower(drivePower);
        backRightWheel.setPower(drivePower);
        backLeftWheel.setPower(drivePower);
    }


}