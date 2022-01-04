package org.firstinspires.ftc.teamcode.Autonomous_WORKING;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@Autonomous
public class RoboticArm3 extends LinearOpMode {
    double IntegralSum = 0;
    double Kp = 0;
    double Ki = 0;
    double Kd = 0;
    DcMotorEx armMotor;
    ElapsedTime timer = new ElapsedTime();
//    public static PIDCoefficients testPID = new PIDCoefficients(0, 0, 0);
    private double lastError = 0;
    FtcDashboard dashboard;

    @Override
    public void runOpMode() throws InterruptedException {
        armMotor = hardwareMap.get(DcMotorEx.class, "expansion_motor");
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        dashboard = FtcDashboard.getInstance();
        waitForStart();
        while (opModeIsActive()) {
            double power = PIDControl(100, armMotor.getCurrentPosition());
            armMotor.setPower(power);

        }
    }
        public double PIDControl(double reference, double state){
            double error = reference - state;
            IntegralSum += error * timer.seconds();
            double derivative = (error - lastError) / timer.seconds();
            lastError = error;
            timer.reset();
            double output = (error * Kp) + (derivative * Kd) + (IntegralSum * Ki);
            return output;
        }

    }
