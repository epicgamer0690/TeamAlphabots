package org.firstinspires.ftc.teamcode.TeleOp_WORKING;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="TeleOpTest_TwoPlayer", group="Training")
    public class TeleOpTest_TwoPlayer extends OpMode {



    DcMotor leftWheel;
    DcMotor rightWheel;
    DcMotor backLeftWheel;
    DcMotor backRightWheel;
    DcMotor armMotor;
    CRServo intakeServo;
    DcMotor carouselMotor;
    double drivePower = 0.5;


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

        leftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);  //setting it so when power is 0, robot stops.

    }

    @Override
    public void loop() {
        moveDriveTrain();
        if(gamepad2.dpad_right){
            shippingHubLevel(65, 1);
        }
        if(gamepad2.dpad_left){
            shippingHubLevel(115, 1 );
        }
        if(gamepad2.dpad_up){
            shippingHubLevel(155, 1);
        }
        if(gamepad2.dpad_down){
            shippingHubLevel(0, 0.2);
        }
        if(gamepad2.cross) {

            carouselMotor.setPower(0.3);

        } else {
            carouselMotor.setPower(0);
        }
        if (gamepad1.right_bumper) {
            intakeServo.setPower(2);
        }
        else if(gamepad1.left_bumper){
            intakeServo.setPower(-2);
        }
        else{
            intakeServo.setPower(0);
        }
    }

    public void shippingHubLevel(int rotation, double power) {
        armMotor.setTargetPosition(rotation);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(power);
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