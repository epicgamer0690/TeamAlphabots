package org.firstinspires.ftc.teamcode.TeleOp_WORKING;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.rev.RevColorSensorV3;

@TeleOp(name="TeleOpTest_TwoPlayer", group="Training")
public class TeleOpTest_TwoPlayer extends OpMode {



    DcMotor leftWheel;
    DcMotor rightWheel;
    DcMotor backLeftWheel;
    DcMotor backRightWheel;
    DcMotor armMotor1;
    DcMotor armMotor2;
    CRServo intakeServo;
    DcMotor carouselMotor;
    RevColorSensorV3 sensorColor;
    double drivePower = 0.5;
    public int count = 0;


    private ElapsedTime runtime= new ElapsedTime();
    private ElapsedTime colorTimer= new ElapsedTime();

    @Override
    public void init() {
        leftWheel = hardwareMap.dcMotor.get("left_wheel");
        rightWheel = hardwareMap.dcMotor.get("right_wheel");
        backRightWheel = hardwareMap.dcMotor.get("back_right_wheel");
        backLeftWheel = hardwareMap.dcMotor.get("back_left_wheel");
        intakeServo = hardwareMap.crservo.get("expansion_servo");
        armMotor1 = hardwareMap.dcMotor.get("expansion_motor1");
        armMotor2 = hardwareMap.dcMotor.get("expansion_motor2");
        carouselMotor = hardwareMap.get(DcMotor.class, "carousel_arm");
        sensorColor = hardwareMap.get(RevColorSensorV3.class, "color_sensor");

        rightWheel.setDirection(DcMotorSimple.Direction.REVERSE); //rightWheel
        backRightWheel.setDirection(DcMotorSimple.Direction.REVERSE); //backRightWheel

        leftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);  //setting it so when power is 0, robot stops.
        armMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        resetArmEncoders();
        armMotor1.setTargetPosition(0);
        armMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor2.setTargetPosition(0);
        armMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);


    }
    @Override
    public void loop() {
        moveDriveTrain();
        if(gamepad2.dpad_left){
            shippingHubLevel(65, 1);
        }
        if(gamepad2.dpad_right){
            shippingHubLevel(115, 1 );
        }
        if(gamepad2.dpad_up){
            shippingHubLevel(155, 1);
        }
        if(gamepad2.dpad_down){
            shippingHubLevel(30,0.2);
            shippingHubLevel(0, 0.2);
        }
        if((sensorColor.red() >= (1.5 * sensorColor.blue())) && (sensorColor.green() >= (1.5 * sensorColor.blue()))) {
            count += 1;
            if(count == 1) {
                gamepad1.rumble(1, 1, 500);
            if (intakeServo.getPower() > 0) {
                sleep(200);
                intakeServo.setPower(0);
            }
            }
        }else{
            count = 0;
        }

        if(gamepad2.cross) {
            carouselMotor.setPower(0.5);
        } else if(gamepad2.circle){
            carouselMotor.setPower(-0.5);
        }
        else {
            carouselMotor.setPower(0);
        }
        if(gamepad2.square){
            resetArmEncoders();
            armMotor1.setTargetPosition(0);
            armMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor2.setTargetPosition(0);
            armMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        if (gamepad1.right_bumper) {
            intakeServo.setPower(2);
        }
        else if(gamepad1.left_bumper){
            intakeServo.setPower(-2);
        }else{
            intakeServo.setPower(0);
        }

        if(runtime.seconds() == 55){
            gamepad2.rumble(1000);
            sleep(1000);
        } else if(runtime.seconds() == 85) {
            gamepad2.rumble(1000);
            sleep(1000);
        }
    }
    public void shippingHubLevel(int rotation, double power) {
        armMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor1.setTargetPosition(rotation);
        armMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor1.setPower(power);
        armMotor2.setTargetPosition(-rotation);
        armMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor2.setPower(power);
    }

    public void sleep(int milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    public void resetArmEncoders(){
        armMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void moveDriveTrain() {
        double driveForward = 0; //Moves forwards and backwards
        double driveBackward = 0;
        double strafe = 0; //Move side-to-side
        double rotate = 0;
        double drive = 0;
        double denominator = 1;

        drive = gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x * 1.1;
        rotate = gamepad1.right_stick_x;

        driveForward = gamepad1.right_trigger;
        driveBackward = gamepad1.left_trigger;

        denominator = Math.max(Math.abs(drive) + Math.abs(strafe) + Math.abs(rotate), 1);
        if(driveForward > 0) {
            rightWheel.setPower(-driveForward);
            backRightWheel.setPower(-driveForward);
            leftWheel.setPower(-driveForward);
            backLeftWheel.setPower(-driveForward);
        } else if(driveBackward > 0) {
            rightWheel.setPower(driveBackward);
            backRightWheel.setPower(driveBackward);
            leftWheel.setPower(driveBackward);
            backLeftWheel.setPower(driveBackward);

        } else {
            rightWheel.setPower((drive + strafe + rotate) / denominator);
            backRightWheel.setPower((drive - rotate + strafe) / denominator);
            leftWheel.setPower((drive - rotate - strafe) / denominator);
            backLeftWheel.setPower((drive + rotate - strafe) / denominator);
        }
    }

}