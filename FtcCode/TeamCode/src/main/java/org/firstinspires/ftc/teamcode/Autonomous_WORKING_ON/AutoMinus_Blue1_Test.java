package org.firstinspires.ftc.teamcode.Autonomous_WORKING_ON;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
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
@Autonomous(name="AutoMinus_Blue1_Test", group="Training")
class AutoMinus_Blue1_Test extends LinearOpMode {

    DcMotor leftWheel;
    DcMotor rightWheel;
    DcMotor backLeftWheel;
    DcMotor backRightWheel;
    DcMotor armMotor;
    DcMotor carouselMotor;
    CRServo intakeServo;
    RevColorSensorV3 colorSensor;
    String TSEPosition;
    final static double ticks_per_cm = 2.0 * Math.PI * 9.6 /537.6;
    private ElapsedTime period = new ElapsedTime();
    @Override
    public void runOpMode() {
        leftWheel = hardwareMap.dcMotor.get("left_wheel");
        rightWheel = hardwareMap.dcMotor.get("right_wheel");
        backRightWheel = hardwareMap.dcMotor.get("back_right_wheel");
        backLeftWheel = hardwareMap.dcMotor.get("back_left_wheel");
        armMotor = hardwareMap.get(DcMotor.class, "expansion_motor");
        carouselMotor = hardwareMap.get(DcMotor.class, "carousel_arm");
        intakeServo = hardwareMap.crservo.get("expansion_servo");
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "color_sensor");

        leftWheel.setDirection(DcMotor.Direction.REVERSE);
        rightWheel.setDirection(DcMotor.Direction.FORWARD);
        backLeftWheel.setDirection(DcMotor.Direction.REVERSE);
        backRightWheel.setDirection(DcMotor.Direction.FORWARD);


        leftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        resetEncoders();
        setCmTargetPosition(10);
        leftWheel.setPower(0.5);
        Sleep(500);
//        rightWheel.setPower(0.5);
//        backRightWheel.setPower(0.5);
//        backLeftWheel.setPower(0.5);
        runToPosition();
        checkBusy();
        stopMotor();













    }

    public void resetEncoders() {
        leftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode((DcMotor.RunMode.STOP_AND_RESET_ENCODER));
        carouselMotor.setMode((DcMotor.RunMode.STOP_AND_RESET_ENCODER));

    }
    public void setCmTargetPosition(double cm) {
        double cm_needed = cm * ticks_per_cm;
        leftWheel.setTargetPosition((int)cm_needed);
        rightWheel.setTargetPosition((int)cm_needed);
        backLeftWheel.setTargetPosition((int)cm_needed);
        backRightWheel.setTargetPosition((int)cm_needed);
    }

    public void runToPosition(){
        leftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void checkBusy() {
        if (opModeIsActive()) {
            while (leftWheel.isBusy() || rightWheel.isBusy() || backLeftWheel.isBusy() || backRightWheel.isBusy()){
                telemetry.addData("Path", "Driving Currently");
                telemetry.update();
            }
        }
    }

    public void stopMotor(){
        leftWheel.setPower(0);
        rightWheel.setPower(0);
        backRightWheel.setPower(0);
        backLeftWheel.setPower(0);

    }

    public void Sleep(int milliseconds){
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

}