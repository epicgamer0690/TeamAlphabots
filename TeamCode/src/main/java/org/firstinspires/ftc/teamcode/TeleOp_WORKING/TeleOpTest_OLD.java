package org.firstinspires.ftc.teamcode.TeleOp_WORKING;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="TeleOpTest_OLD", group="Training")
    public class TeleOpTest_OLD extends OpMode {



    DcMotor leftWheel;
    DcMotor rightWheel;
    DcMotor backLeftWheel;
    DcMotor backRightWheel;
    DcMotor armMotor;
    CRServo intakeServo;
    DcMotor carouselMotor;
    double drivePower = 0.5;


    private ElapsedTime runtime= new ElapsedTime();




    public void spin() {
        double pivot = 0;
        pivot = gamepad1.right_stick_y;;
        if(pivot < 0) {
            rightWheel.setPower(-pivot);
            backRightWheel.setPower(-pivot);
            leftWheel.setPower(pivot);
            backLeftWheel.setPower(pivot);
        }
        if(pivot > 0) {
            rightWheel.setPower(-pivot);
            backRightWheel.setPower(-pivot);
            leftWheel.setPower(pivot);
            backLeftWheel.setPower(pivot);
        }
    }




    public void moveDriveTrain() {

        double vertical = -gamepad1.left_stick_y;
        double peevot = gamepad1.left_stick_x;
        double horizontal = gamepad1.right_stick_x;
        double driveForward = gamepad1.right_trigger;
        double driveBackward = gamepad1.left_trigger;

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

        }
        else {
            rightWheel.setPower(peevot + (-vertical + horizontal));
            backRightWheel.setPower(peevot + (-vertical - horizontal));
            leftWheel.setPower(peevot + (-vertical - horizontal));
            backLeftWheel.setPower(peevot + (-vertical + horizontal));
        }

        spin();
    }



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
        setButtons();
    }

    public void setButtons(){
        if(gamepad1.dpad_right){
            shippingHubLevel(65, 1);
        }
        if(gamepad1.dpad_left){
            shippingHubLevel(115, 1 );
        }
        if(gamepad1.dpad_up){
            shippingHubLevel(155, 1);
        }
        if(gamepad1.dpad_down){
            shippingHubLevel(0, 0.2);
        }
        if(gamepad1.cross) {

            carouselMotor.setPower(0.3);

        } else {
            carouselMotor.setPower(0);
        }
        if (gamepad1.left_bumper) {
            intakeServo.setPower(2);
        }
        else if(gamepad1.right_bumper){
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

    /*
    public void accelerate(){
        if(gamepad2.right_bumper == true){
            drivePower = drivePower + 0.25;
        }
        else{
            drivePower = 0.5;
        }
    }
    public void deccelerate(){
        if(gamepad2.left_bumper == true){
            drivePower = drivePower - 0.25;
        }
        else{
            drivePower = 0.5;
        }
    }


    public void stopMotors(){
        if(gamepad2.square == true){
            drivePower = 0;
        }
        else{
            drivePower = 0.5;
        }
    }

     */

}