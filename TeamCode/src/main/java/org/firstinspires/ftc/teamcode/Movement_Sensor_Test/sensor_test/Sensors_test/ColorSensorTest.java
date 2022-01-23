package org.firstinspires.ftc.teamcode.Movement_Sensor_Test.sensor_test.Sensors_test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.rev.RevColorSensorV3;

@Autonomous(name="ColorSensorTest", group="Training")
    public class ColorSensorTest extends LinearOpMode {



    DcMotor leftWheel;
    DcMotor rightWheel;
    DcMotor backLeftWheel;
    DcMotor backRightWheel;
    RevColorSensorV3 sensorColor;
    double drivePower = 0.5;
     //1 rotation = 360












    private ElapsedTime runtime= new ElapsedTime();

    @Override
    public void runOpMode() {
        leftWheel = hardwareMap.dcMotor.get("left_wheel");
        rightWheel = hardwareMap.dcMotor.get("right_wheel");
        backRightWheel = hardwareMap.dcMotor.get("back_right_wheel");
        backLeftWheel = hardwareMap.dcMotor.get("back_left_wheel");
        sensorColor = hardwareMap.get(RevColorSensorV3.class, "color_sensor");
        sensorColor.enableLed(true);


        waitForStart();
        while(opModeIsActive()) {
            if((sensorColor.red() >= (1.5 * sensorColor.blue())) && (sensorColor.green() >= (1.5 * sensorColor.blue()))) {
                telemetry.addData("Yellow", true);
                telemetry.update();
            } else {
                telemetry.addData("Yellow", false);
                telemetry.update();

            }




        }
    }



    public double averageRed() {
        int sum = 0;
        int count = 0;
        for(int i = 0; i < 101; i++) {
            sum += sensorColor.red();
            count++;
            sleep(100);
        }
        return (double) sum/count;
    }

    public double averageGreen() {
        int sum = 0;
        int count = 0;
        for(int i = 0; i < 101; i++) {
            sum += sensorColor.green();
            count++;
        }
        return (double) sum/count;
    }


    public double averageBlue() {
        int sum = 0;
        int count = 0;
        for(int i = 0; i < 101; i++) {
            sum += sensorColor.blue();
            count++;
        }
        return (double) sum/count;
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




    }