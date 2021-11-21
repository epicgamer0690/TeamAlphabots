package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="TeleOpTest", group="Training")
    public class TeleOpTest extends OpMode {



    DcMotor leftWheel;
    DcMotor rightWheel;
    DcMotor backLeftWheel;
    DcMotor backRightWheel;
    DcMotor armMotor;
    CRServo intakeServo;
    DcMotor carouselMotor;
    double drivePower = 0.5;
    int rotation = 1000; //1 rotation = 360

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
        double vertical = 0; //Moves forwards and backwards
        double horizontal = 0; //Move side-to-side
        double peevot = 0;

        vertical = -gamepad1.left_stick_y;
        horizontal = gamepad1.left_stick_x;
        peevot = gamepad1.right_stick_x;
        rightWheel.setPower(peevot + (-vertical + horizontal));
        backRightWheel.setPower(peevot + (-vertical - horizontal));
        leftWheel.setPower(peevot + (-vertical - horizontal));
        backLeftWheel.setPower(peevot + (-vertical + horizontal));

        spin();
    }



    @Override
    public void init() {
        leftWheel = hardwareMap.dcMotor.get("left_wheel");
        rightWheel = hardwareMap.dcMotor.get("right_wheel");
        backRightWheel = hardwareMap.dcMotor.get("back_right_wheel");
        backLeftWheel = hardwareMap.dcMotor.get("back_left_wheel");
        intakeServo = hardwareMap.crservo.get("intake_servo");
        armMotor = hardwareMap.dcMotor.get("expansion_motor");
        carouselMotor = hardwareMap.get(DcMotor.class, "carousel_arm");

        rightWheel.setDirection(DcMotorSimple.Direction.REVERSE); //rightWheel
        backRightWheel.setDirection(DcMotorSimple.Direction.REVERSE); //backRightWheel

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

    @Override
    public void start() {



    }

    @Override
    public void loop() {
        moveDriveTrain();
        intakeFunc();
        outakeFunc();
        setButtons();

    }
    @Override
    public void stop() {
        leftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


    }





    //@Override
    //public void loop() {
    //    leftWheel.setPower(drivePower);
    //    rightWheel.setPower(drivePower);
    //    backRightWheel.setPower(drivePower);
    //    backLeftWheel.setPower(drivePower);


//    }
    public void setButtons(){
        if(gamepad1.dpad_left == true){
            shippingHubLevel(65);
        }
        if(gamepad1.dpad_right == true){
            shippingHubLevel(125 );
        }
        if(gamepad1.dpad_up == true){
            shippingHubLevel(195);
        }
        if(gamepad1.triangle == true){
            shippingHubLevelReturn(195);
        }
        if(gamepad1.circle == true){
            shippingHubLevelReturn(125);
        }
        if(gamepad1.square == true){
            shippingHubLevelReturn(65);
        }
        if(gamepad1.cross == true){
            carouselFunc(7);
        }
    }
    public void shippingHubLevel(int rotation) {
        armMotor.setTargetPosition(rotation);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(1);
    }
    public void shippingHubLevelReturn(int rotation){
        armMotor.setTargetPosition(-rotation);
        armMotor.setPower(0.04);
    }
    public void carouselFunc(int rotation){
            carouselMotor.setTargetPosition(-rotation);
        carouselMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            carouselMotor.setPower(0.5);
    }

    public void intakeFunc() {
        boolean inteeke;
        inteeke = gamepad1.left_bumper;
        if (inteeke == true) {
            intakeServo.setPower(1);
        }
        else{
            intakeServo.setPower(0);
        }
    }

    public void outakeFunc(){
        boolean outeeke;
        outeeke = gamepad1.right_bumper;
        if(outeeke == true) {
            intakeServo.setPower(-1);
        }
        else{
            intakeServo.setPower(0);
        }
    }
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


    public void diagonalLeft() {
        /*
        backLeftWheel.setDirection(DcMotor.Direction.REVERSE);
        rightWheel.setDirection(DcMotor.Direction.FORWARD);

        rightWheel.setPower(drivePower);
        backLeftWheel.setPower(drivePower);
        //sleep(milliseconds 1000);

         */

        rightWheel.setTargetPosition(rotation);
        backLeftWheel.setTargetPosition(-rotation);

        rightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightWheel.setPower(drivePower);
        backLeftWheel.setPower(-drivePower);



    }

    public void backwardsDiagonalLeft() {
        /*
        backLeftWheel.setDirection(DcMotor.Direction.FORWARD);
        rightWheel.setDirection(DcMotor.Direction.REVERSE);

        rightWheel.setPower(drivePower);
        backLeftWheel.setPower(drivePower);
        //sleep(2000);

         */

        rightWheel.setTargetPosition(-rotation);
        backLeftWheel.setTargetPosition(rotation);

        rightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightWheel.setPower(-drivePower);
        backLeftWheel.setPower(drivePower);

    }

    public void diagonalRight() {
        /*
        backRightWheel.setDirection(DcMotor.Direction.FORWARD);
        leftWheel.setDirection(DcMotor.Direction.REVERSE);

        leftWheel.setPower(drivePower);
        backRightWheel.setPower(drivePower);

         */

        leftWheel.setTargetPosition(-rotation);
        backRightWheel.setTargetPosition(rotation);

        leftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftWheel.setPower(-drivePower);
        backRightWheel.setPower(drivePower);
    }

    public void right() {

        leftWheel.setTargetPosition(-rotation);
        rightWheel.setTargetPosition(-rotation);
        backLeftWheel.setTargetPosition(rotation);
        backRightWheel.setTargetPosition(rotation);

        leftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftWheel.setPower(-drivePower);
        rightWheel.setPower(-drivePower);
        backLeftWheel.setPower(drivePower);
        backRightWheel.setPower(drivePower);
    }

    public void left() {

        leftWheel.setTargetPosition(rotation);
        rightWheel.setTargetPosition(rotation);
        backLeftWheel.setTargetPosition(-rotation);
        backRightWheel.setTargetPosition(-rotation);

        leftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftWheel.setPower(drivePower);
        rightWheel.setPower(drivePower);
        backLeftWheel.setPower(-drivePower);
        backRightWheel.setPower(-drivePower);
    }

    public void backwardsDiagonalRight() {

        leftWheel.setTargetPosition(-rotation);
        backRightWheel.setTargetPosition(rotation);

        leftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftWheel.setPower(-drivePower);
        backRightWheel.setPower(drivePower);

    }



    public void forward() {


        leftWheel.setTargetPosition(-rotation);
        rightWheel.setTargetPosition(rotation);
        backLeftWheel.setTargetPosition(-rotation);
        backRightWheel.setTargetPosition(rotation);

        leftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftWheel.setPower(-drivePower);
        rightWheel.setPower(drivePower);
        backLeftWheel.setPower(-drivePower);
        backRightWheel.setPower(drivePower);

        leftWheel.setTargetPosition(-300);
        rightWheel.setTargetPosition(300);
        backLeftWheel.setTargetPosition(-300);
        backRightWheel.setTargetPosition(300);

        leftWheel.setPower(-0.2);
        rightWheel.setPower(0.2);
        backLeftWheel.setPower(-0.2);
        backRightWheel.setPower(0.2);



    }

    public void backward() {


        leftWheel.setTargetPosition(rotation);
        rightWheel.setTargetPosition(-rotation);
        backLeftWheel.setTargetPosition(rotation);
        backRightWheel.setTargetPosition(-rotation);

        leftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftWheel.setPower(drivePower);
        rightWheel.setPower(-drivePower);
        backLeftWheel.setPower(drivePower);
        backRightWheel.setPower(-drivePower);



    }






}