package org.firstinspires.ftc.teamcode.Autonomous_WORKING_ON;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="AutoMinus", group="Training")
    public class AutoMinus_Blue1 extends OpMode {

    DcMotor leftWheel;
    DcMotor rightWheel;
    DcMotor backLeftWheel;
    DcMotor backRightWheel;
    DcMotor armMotor;
    DcMotor carouselMotor;
    CRServo intakeServo;
    RevColorSensorV3 colorSensor;
    String TSEPosition;
    double drivePower = 0.5;
    //1 rotation = 360

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init() {
        leftWheel = hardwareMap.dcMotor.get("left_wheel");
        rightWheel = hardwareMap.dcMotor.get("right_wheel");
        backRightWheel = hardwareMap.dcMotor.get("back_right_wheel");
        backLeftWheel = hardwareMap.dcMotor.get("back_left_wheel");
        armMotor = hardwareMap.get(DcMotor.class, "expansion_motor");
        carouselMotor = hardwareMap.get(DcMotor.class, "carousel_arm");
        intakeServo = hardwareMap.crservo.get("expansion_servo");
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "color_sensor");


    }

    public void sleep(int milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    public void shippingHubLevel(int rotation) {
        resetEncoders();
        armMotor.setTargetPosition(rotation);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(1);
    }

    public void shippingHubLevelReturn(int rotation){
        resetEncoders();
        armMotor.setTargetPosition(-rotation);
        armMotor.setPower(0.04);
    }

    public void intakeFunc() {
        intakeServo.setPower(1);
    }

    public void outakeFunc(){
        intakeServo.setPower(-1);
        sleep(3000);
        }


    public void carouselFunc(double power){
        carouselMotor.setTargetPosition(6);
        carouselMotor.setPower(1);
    }

    public void resetEncoders() {
        leftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode((DcMotor.RunMode.STOP_AND_RESET_ENCODER));
        carouselMotor.setMode((DcMotor.RunMode.STOP_AND_RESET_ENCODER));

    }
    public int detectShippingElement() {
        int tempVar = 0;
        if (colorSensor.green() > colorSensor.red() && colorSensor.green() > colorSensor.blue()) {
            forward(1); //to the next square
            tempVar = 3;
        } else if (colorSensor.green() <= colorSensor.red() && colorSensor.green() <= colorSensor.blue()) {
            forward(1); //to the next square
            if (colorSensor.green() > colorSensor.red() && colorSensor.green() > colorSensor.blue()) {
                tempVar = 2;
            } else {
                tempVar = 1;
            }

        }
        return tempVar;
    }

    public void moveArm() {
        if(detectShippingElement() == 1) {
            shippingHubLevel(65);
            outakeFunc();
        }
        if(detectShippingElement() == 2) {
            forward(1);
            shippingHubLevel(125);
            outakeFunc();
            backward(1);
        }
        if(detectShippingElement() == 3) {
            forward(2);
            shippingHubLevel(195);
            outakeFunc();
            backward(2);
        }
    }

    public void Sleep(int milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    public void pivotRight(int rotation) {
        rightWheel.setTargetPosition(-rotation);
        backLeftWheel.setTargetPosition(rotation);
        backRightWheel.setTargetPosition(-rotation);

        rightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightWheel.setPower(-drivePower);
        backLeftWheel.setPower(drivePower);
        backRightWheel.setPower(-drivePower);

    }

    public boolean busyness(int wheelNumber) {
        if(wheelNumber == 1) {
            return leftWheel.isBusy();
        }
        if(wheelNumber == 2) {
            return rightWheel.isBusy();
        }
        if(wheelNumber == 3){
            return backLeftWheel.isBusy();
        }
        if(wheelNumber == 4) {
            return backRightWheel.isBusy();
        }
        return false;
    }

    public void stopMotor() {
        leftWheel.setPower(0);
        rightWheel.setPower(0);
        backRightWheel.setPower(0);
        backLeftWheel.setPower(0);

    }

    @Override
    public void start() {

        //BLUE 1: strafe to barcode (Hor Right). Sense TSE. Forward. Drop cargo based on TSE. Move back to wall. Turn 90 degrees forward. Forward to carousel. Turn carousel. MOVE BACK TO THE BLUE TAPE AREA
        //BLUE 2: strafe to barcode (Hor Left). Sense TSE. Forward. Drop cargo based on TSE. Move Back;
        /*
        horizontalRight(5);
        resetEncoders();
        detectShippingElement
        backwards(0.2)
        horizontalRight(0.2)
        moveArm();
        forward(5);
        backward(10);
        pivotRight(1);
        backwards(2);
        carouselFunc(1)
        forwards(2)
         */
        int Motor_Tick_Counts = 558;
        double distance = 96;
        double rotationsNeeded = distance/(3.14 * 96);
        int target = (int)(rotationsNeeded) * Motor_Tick_Counts;
        forward(target);
        while(busyness(1) || busyness(2) || busyness(3) || busyness(4)) {
            telemetry.addData("Path", "Driving");
            telemetry.update();
        }
        stopMotor();
        resetEncoders();


    }

    @Override
    public void loop() {
    }

    @Override
    public void stop() {
        leftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


    }



    public void diagonalLeft(int rotation) {
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

    public void backwardsDiagonalLeft(int rotation) {
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

    public void diagonalRight(int rotation) {
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

    public void horizontalRight(int rotation) {
        backRightWheel.setDirection(DcMotor.Direction.FORWARD);
        leftWheel.setDirection(DcMotor.Direction.REVERSE);
        rightWheel.setDirection(DcMotor.Direction.REVERSE);
        backLeftWheel.setDirection(DcMotor.Direction.FORWARD);

        leftWheel.setTargetPosition(rotation);
        rightWheel.setTargetPosition(rotation);
        backLeftWheel.setTargetPosition(rotation);
        backRightWheel.setTargetPosition(rotation);

        leftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftWheel.setPower(drivePower);
        rightWheel.setPower(drivePower);
        backLeftWheel.setPower(drivePower);
        backRightWheel.setPower(drivePower);
    }

    public void horizontalLeft(int rotation) {

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

    public void backwardsDiagonalRight(int rotation) {

        leftWheel.setTargetPosition(rotation);
        backRightWheel.setTargetPosition(-rotation);

        leftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftWheel.setPower(drivePower);
        backRightWheel.setPower(-drivePower);

    }


    public void forward(int rotation) {


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


    }

    public void backward(int rotation) {


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