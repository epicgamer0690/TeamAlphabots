package org.firstinspires.ftc.teamcode.Autonomous_WORKING;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ArmFeedforward;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImpl;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@Disabled
@Autonomous(name="RoboticArmTest", group="Training")
public class RoboticArmTest extends LinearOpMode {

    DcMotorImplEx armMotor;

    public static double Kp = 0;
    public static double Ki = 0;
    public static double Kd = 0;
    public static double targetPosition = 0;
    public static double velocity = 0;
    double integralSum = 0;
    private double lastError = 0;
    double tolerance = 5;
    public static ArmFeedforward feedforward = new ArmFeedforward(0, 0, 0, 0);
    ElapsedTime runtime = new ElapsedTime();
    FtcDashboard dashboard;
    double ticksPerRev = 537.6;

    public void runOpMode() {
        //init
        armMotor = hardwareMap.get(DcMotorImplEx.class, "expansion_motor");
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setMode(DcMotorImplEx.RunMode.RUN_WITHOUT_ENCODER);
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //start
        waitForStart();
        runtime.startTime();
        while(opModeIsActive()) {
            PIDController(targetPosition, velocity);

        }
    }

    public double PIDF(double targetPos, double targetVel) {
        double power = feedforward.calculate(targetPos, targetVel);
        return power;
    }
    public void PIDController(double targetPosition, double targetVelocity) {
        while(Math.abs(targetPosition - armMotor.getCurrentPosition()) > 5) {

            double targetEncoderPosition = targetPosition;
            telemetry.addData("Target Position", targetEncoderPosition);
            telemetry.update();

            double encoderPosition = armMotor.getCurrentPosition();
            telemetry.addData("Encoder Position", encoderPosition);
            telemetry.update();

            double error = targetEncoderPosition - encoderPosition;

            if(Math.abs(error) <= tolerance) {
                break;
            }

            double derivative = (error - lastError) / runtime.seconds();

            integralSum += (error * runtime.seconds());
            double KpError = Kp * error;
            telemetry.addData("kP Error", KpError);
            telemetry.update();
            double KiIntegral = Ki * integralSum;
            telemetry.addData("kI Integral", KiIntegral);
            telemetry.update();
            double KdDerivative = Kd * derivative;
            telemetry.addData("kD Derivative", KdDerivative);
            telemetry.update();

            double output = KpError + KiIntegral + KdDerivative + PIDF(targetPosition, targetVelocity);

            lastError = error;

            armMotor.setPower(output);

            runtime.reset();



        }

    }


}
