package org.firstinspires.ftc.teamcode.Movement_Sensor_Test.sensor_test.Sensors_test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.TouchSensor;

@Disabled
@Autonomous(name="TouchSensorTest", group="Training")
    public class TouchSensorTest extends OpMode {


    TouchSensor touch;
    /*
    DcMotor leftWheel;
    DcMotor rightWheel;
    DcMotor backLeftWheel;
    DcMotor backRightWheel;

     */
    double drivePower = 0.5;
     //1 rotation = 360





    private ElapsedTime runtime= new ElapsedTime();

    @Override
    public void init() {
        touch = hardwareMap.get(TouchSensor.class, "touch_sensor");

    }

    public void Sleep(int milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
    /*
    public void resetEncoders() {
        leftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

     */

    @Override
    public void start() {

    }

    @Override
    public void loop() {

        if(touch.isPressed()) {
            telemetry.addData("Button is Pressed:", true);
        } else {
            telemetry.addData("Button is Pressed:", false);
        }
        telemetry.update();
    }
    @Override
    public void stop() {



    }





}