package org.firstinspires.ftc.teamcode.Autonomous_WORKING;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@Autonomous(name="RoboticArm2", group="Training")
public class RoboticArm2 extends LinearOpMode {





        /**
         * make sure to make sure your the code matches your configuration
         */
        private DcMotorEx armMotor;

        public static double speed = 100; //arbitrary number; static to allow for analyzing how PID performs through multiple speeds in dashboard
        FtcDashboard dashboard;
        public static PIDCoefficients pidCoeffs = new PIDCoefficients(0, 0, 0); //PID coefficients that need to be tuned probably through FTC dashboard
        public PIDCoefficients pidGains = new PIDCoefficients(0, 0, 0); //PID gains which we will define later in the process

        ElapsedTime PIDTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS); //timer


        @Override
        public void runOpMode() {
            /**
             * basic initialization stuff needs to be changed to suit your configuration (motor name, direction, etc.)
             */
            armMotor = hardwareMap.get(DcMotorEx.class, "expansion_motor");

            dashboard = FtcDashboard.getInstance();
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

            armMotor.setDirection(DcMotorSimple.Direction.REVERSE);

            armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            waitForStart();

            if (opModeIsActive()) {
                while (opModeIsActive()) {

                     //running the PID algorithm at defined speed
                    PID(speed);
                    telemetry.addData("Target Position", speed);
                    telemetry.update();
                }
            }
        }

        double lastError = 0;
        double integral = 0;
        //initializing our variables

        public void PID(double targetVelocity){
            PIDTimer.reset(); //resets the timer

            double currentVelocity = armMotor.getVelocity();
            telemetry.addData("Current Position", currentVelocity);
            telemetry.update();
            double error = targetVelocity - currentVelocity; //pretty self explanatory--just finds the error

            double deltaError = error - lastError; //finds how the error changes from the previous cycle
            double derivative = deltaError / PIDTimer.time(); //deltaError/time gives the rate of change (sensitivity of the system)

            integral += error * PIDTimer.time();
            //continuously sums error accumulation to prevent steady-state error (friction, not enough p-gain to cause change)

            pidGains.p = error * pidCoeffs.p;
            //acts directly on the error; p-coefficient identifies how much to act upon it
            // p-coefficient (very low = not much effect; very high = lots of overshoot/oscillations)
            pidGains.i = integral * pidCoeffs.i;
            //multiplies integrated error by i-coefficient constant
            // i-coefficient (very high = fast reaction to steady-state error but lots of overshoot; very low = slow reaction to steady-state error)
            // for velocity, because friction isn't a big issue, only reason why you would need i would be for insufficient correction from p-gain
            pidGains.d = derivative * pidCoeffs.d;
            //multiplies derivative by d-coefficient
            // d-coefficient (very high = increased volatility; very low = too little effect on dampening system)

            armMotor.setVelocity(pidGains.p + pidGains.i + pidGains.d + targetVelocity);
            //adds up the P I D gains with the targetVelocity bias

            lastError = error;
            //makes our current error as our new last error for the next cycle

    }
}
