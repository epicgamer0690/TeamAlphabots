package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="TeleOpTest", group="Training")
    public class TeleOpTest extends OpMode {



    DcMotor leftWheel;
    DcMotor rightWheel;
    DcMotor backLeftWheel;
    DcMotor backRightWheel;
    double drivePower = 0.5;
    int rotation = 1000; //1 rotation = 360





    private ElapsedTime runtime= new ElapsedTime();

    public void spin() {
        double pivot = 0;
        pivot = gamepad1.right_stick_x;;
        if(pivot < 0) {
            rightWheel.setPower(1);
            backRightWheel.setPower(1);
            leftWheel.setPower(-1);
            backLeftWheel.setPower(-1);
        }
        if(pivot > 0) {
            rightWheel.setPower(-1);
            backRightWheel.setPower(-1);
            leftWheel.setPower(1);
            backLeftWheel.setPower(1);
        }
    }

    public void moveDriveTrain() {
        double vertical = 0; //Moves forwards and backwards
        double horizontal = 0; //Move side-to-side
        double pivot = 0;
        vertical = -gamepad1.left_stick_y;
        horizontal = gamepad1.left_stick_x;

        rightWheel.setPower(pivot + (-vertical + horizontal));
        backRightWheel.setPower(pivot + (-vertical - horizontal));
        leftWheel.setPower(pivot + (-vertical - horizontal));
        backLeftWheel.setPower(pivot + (-vertical + horizontal));

        spin();
    }



    @Override
    public void init() {
        leftWheel = hardwareMap.dcMotor.get("left_wheel");
        rightWheel = hardwareMap.dcMotor.get("right_wheel");
        backRightWheel = hardwareMap.dcMotor.get("back_right_wheel");
        backLeftWheel = hardwareMap.dcMotor.get("back_left_wheel");

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