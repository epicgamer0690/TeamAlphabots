package org.firstinspires.ftc.teamcode.Movement_Sensor_Test.sensor_test.Sensors_test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.rev.RevColorSensorV3;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

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
//            if((sensorColor.red() + sensorColor.green()) > 3.0 * sensorColor.blue()) {
//                telemetry.addData("Yellow", true);
//                telemetry.update();
//            } else {
//                telemetry.addData("Yellow", false);
//                telemetry.update();
//
//            }


        }
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