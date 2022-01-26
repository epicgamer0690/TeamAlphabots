package org.firstinspires.ftc.teamcode.Movement_Sensor_Test.sensor_test.Sensors_test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
@Disabled
@Autonomous(name="Intake Test", group="Training")
public class IntakeTest extends OpMode {


    CRServo intake_servo;

    @Override
    public void init() {
        intake_servo = hardwareMap.crservo.get("expansion_servo");


    }

    @Override
    public void loop() {

        intakeFunc(-2, 3000);


    }

    public void sleep(int milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }


    public void intakeFunc(double pwr, int time) {
        intake_servo.setDirection(CRServo.Direction.REVERSE);
        intake_servo.setPower(pwr);
        sleep(time);
        intake_servo.setPower(0);

    }
}





