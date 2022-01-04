package org.firstinspires.ftc.teamcode.Autonomous_WORKING;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImpl;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;


import javax.tools.ForwardingFileObject;

@Autonomous(name="RoboticArm", group="Training")
public class RoboticArm  extends LinearOpMode {



    DcMotor leftWheel;
    DcMotor rightWheel;
    DcMotor backLeftWheel;
    DcMotor backRightWheel;
    DcMotor armMotor;
    double integralSum = 0;
    double Kp = 0.005;
    double Ki = 0;
    double Kd = 0;

    private double lastError;

    private ElapsedTime timer = new ElapsedTime();









    @Override
    public void runOpMode() throws InterruptedException {
        armMotor = hardwareMap.get(DcMotor.class, "expansion_motor");
        shippingHubLevelPID(3);


    }


    public void sleep(int milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    public void resetEncoders(){
        leftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }


    public double PIDControl(double reference, double state){
        double error = reference - state;
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) * timer.seconds();
        lastError = error;

        timer.reset();

        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki);
        return(output);
    }


    public void shippingHubLevelPID(int shippingLevel) {
        resetEncoders();
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
        while((Math.abs(reference - armMotor.getCurrentPosition()) > 5) && opModeIsActive()){
            armMotor.setPower(PIDControl(reference, armMotor.getCurrentPosition()));
        }
    }
}