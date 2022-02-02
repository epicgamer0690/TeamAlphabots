package org.firstinspires.ftc.teamcode.TeleOp_WORKING;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Autonomous_WORKING.AutoMinus_Blue2;
@TeleOp(name="TeleOpTest_CarouselDebug", group="Training")
public class TeleOpTest_CarouselDebug extends OpMode {



    DcMotor leftWheel;
    DcMotor rightWheel;
    DcMotor backLeftWheel;
    DcMotor backRightWheel;
    DcMotor armMotor;
    CRServo intakeServo;
    DcMotor carouselMotor;
    BNO055IMU imu;
    private Orientation lastAngles = new Orientation();
    private double currAngle = 0.0;
    public boolean IsStopRequested = false;
    static double pwr = 0.3;
    static int msTime = 700;

    AutoMinus_Blue2 robot = new AutoMinus_Blue2();
    private ElapsedTime runtime= new ElapsedTime();

    @Override
    public void init() {
        leftWheel = hardwareMap.dcMotor.get("left_wheel");
        rightWheel = hardwareMap.dcMotor.get("right_wheel");
        backRightWheel = hardwareMap.dcMotor.get("back_right_wheel");
        backLeftWheel = hardwareMap.dcMotor.get("back_left_wheel");
        intakeServo = hardwareMap.crservo.get("expansion_servo");
        armMotor = hardwareMap.dcMotor.get("expansion_motor");
        carouselMotor = hardwareMap.get(DcMotor.class, "carousel_arm");

        rightWheel.setDirection(DcMotorSimple.Direction.REVERSE); //rightWheel
        backRightWheel.setDirection(DcMotorSimple.Direction.REVERSE); //backRightWheel

        leftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);  //setting it so when power is 0, robot stops.

    }

    @Override
    public void loop() {
        if(gamepad1.left_bumper){
            carouselMotor.setPower(pwr);
            telemetry.addData("power", pwr);
            telemetry.update();
        }
        if(gamepad1.dpad_up){
            pwr += 0.1;
            telemetry.addData("power", pwr);
            telemetry.update();
        }
        if(gamepad1.dpad_down){
            pwr -= 0.1;
            telemetry.addData("power", pwr);
            telemetry.update();
        }
        if(gamepad1.triangle){
            msTime += 100;
            telemetry.addData("speed (Ms)", msTime);
            telemetry.update();
        }
        if(gamepad1.cross){
            msTime -= 100;
            telemetry.addData("speed (Ms)", msTime);
            telemetry.update();
        }
        if(gamepad1.right_bumper){
            carouselMotor.setPower(0.2);
            sleep(msTime);
            carouselMotor.setPower(1);
            sleep(1500);

        }
    }

    public void sleep(int milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}