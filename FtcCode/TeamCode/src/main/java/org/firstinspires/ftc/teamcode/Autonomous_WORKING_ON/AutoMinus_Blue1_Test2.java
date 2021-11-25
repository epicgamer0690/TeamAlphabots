
package org.firstinspires.ftc.teamcode.Autonomous_WORKING_ON;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="AutoMinus_Blue1_Test2", group="Training")
public class AutoMinus_Blue1_Test2 extends OpMode {

    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor rearLeftMotor;
    DcMotor rearRightMotor;
    DcMotor armMotor;
    DcMotor carouselMotor;
    CRServo intakeServo;
    RevColorSensorV3 colorSensor;
    double drivePower = 0.5;
    //1 rotation = 360

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init() {
        frontLeftMotor = hardwareMap.dcMotor.get("left_wheel");
        frontRightMotor = hardwareMap.dcMotor.get("right_wheel");
        rearRightMotor = hardwareMap.dcMotor.get("back_right_wheel");
        rearLeftMotor = hardwareMap.dcMotor.get("back_left_wheel");
        armMotor = hardwareMap.get(DcMotor.class, "expansion_motor");
        carouselMotor = hardwareMap.get(DcMotor.class, "carousel_arm");
        intakeServo = hardwareMap.crservo.get("expansion_servo");
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "color_sensor");


    }

    @Override
    public void start() {
        encoderDriveMat(200, 1, 0.2);
        encoderDriveMat(100, 2, 0.2);
    }

    @Override
    public void loop() {

    }

    public void encoderDriveMat(double distance, int direction, double power) {
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        final double ENCODER_CPR = 537.6;
        final double GEAR_RATIO = 1;
        final double WHEEL_DIAMETER = 9.6;
        final double CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
        double ROTATIONS = distance / CIRCUMFERENCE;
        double counts = ENCODER_CPR * ROTATIONS * GEAR_RATIO;

        switch (direction) {
            case 1: // robot will move forward
                frontLeftMotor.setTargetPosition((int) counts);
                frontRightMotor.setTargetPosition((int) counts);
                rearLeftMotor.setTargetPosition((int) counts);
                rearRightMotor.setTargetPosition((int) counts);
                break;
            case 2: // robot will move backward
                frontLeftMotor.setTargetPosition((int) -counts);
                frontRightMotor.setTargetPosition((int) -counts);
                rearLeftMotor.setTargetPosition((int) -counts);
                rearRightMotor.setTargetPosition((int) -counts);
                break;
            case 3: // robot will strafe left
                frontLeftMotor.setTargetPosition((int) -counts);
                frontRightMotor.setTargetPosition((int) counts);
                rearLeftMotor.setTargetPosition((int) counts);
                rearRightMotor.setTargetPosition((int) -counts);
                break;
            case 4: // robot will strafe right
                frontLeftMotor.setTargetPosition((int) counts);
                frontRightMotor.setTargetPosition((int) -counts);
                rearLeftMotor.setTargetPosition((int) -counts);
                rearRightMotor.setTargetPosition((int) counts);
                break;
            case 5: // robot will rotate left
                frontLeftMotor.setTargetPosition((int) -counts);
                frontRightMotor.setTargetPosition((int) counts);
                rearLeftMotor.setTargetPosition((int) -counts);
                rearRightMotor.setTargetPosition((int) counts);
                break;
            case 6: // robot will rotate right
                frontLeftMotor.setTargetPosition((int) counts);
                frontRightMotor.setTargetPosition((int) -counts);
                rearLeftMotor.setTargetPosition((int) counts);
                rearRightMotor.setTargetPosition((int) -counts);
                break;
        }
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        while (frontLeftMotor.isBusy() && frontRightMotor.isBusy() && rearLeftMotor.isBusy() && rearRightMotor.isBusy()) {
            frontLeftMotor.setPower(power);
            frontRightMotor.setPower(power);
            rearLeftMotor.setPower(power);
            rearRightMotor.setPower(power);
        }
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
