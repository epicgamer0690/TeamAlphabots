package org.firstinspires.ftc.teamcode.TeleOp_WORKING;

import com.qualcomm.hardware.bosch.BNO055IMU;
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
        if(gamepad1.cross){
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
    }
    public void stop(){
        IsStopRequested = true;
    }

    public void shippingHubLevel(int rotation, double power) {
        armMotor.setTargetPosition(rotation);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(power);
    }

    public void moveDriveTrain() {
        double strafe = 0; //Move side-to-side
        double rotate = 0;
        double drive = 0;
        double denominator = 1;

        drive = gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x * 1.1;
        rotate = gamepad1.right_stick_x;

        denominator = Math.max(Math.abs(drive) + Math.abs(strafe)
        + Math.abs(rotate), 1);

        rightWheel.setPower((drive + strafe + rotate) / denominator);
        backRightWheel.setPower((drive - rotate + strafe) / denominator);
        leftWheel.setPower((drive - rotate - strafe) / denominator);
        backLeftWheel.setPower((drive + rotate - strafe) / denominator);
    }
    public void encoderMovement(double distance, int direction, double power) {

        resetEncoders();
        setRUE();

        leftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        final double ENCODER_TPR = 537.6;
        final double GEAR_RATIO = 1;
        final double WHEEL_DIAMETER = 9.6;
        final double CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
        final double ROTATIONS = distance / CIRCUMFERENCE;
        double ticks = ENCODER_TPR * ROTATIONS * GEAR_RATIO;

        switch (direction) {
            case 1: // robot will move forward
                setTargetPositionCounts(ticks, ticks, ticks, ticks);
                setAllMotorPowers(power);
                break;
            case 2: // robot will move backward
                setTargetPositionCounts(-ticks, -ticks, -ticks, -ticks);
                setAllMotorPowers(power);

                break;
            case 3: // robot will strafe left
                setTargetPositionCounts(-ticks, ticks, ticks, -ticks);
                setAllMotorPowers(power);
                break;
            case 4: // robot will strafe right
                setTargetPositionCounts(ticks, -ticks, -ticks, ticks);
                setAllMotorPowers(power);
                break;
            case 5: // robot will rotate left
                setTargetPositionCounts(-ticks, ticks, -ticks, ticks);
                setAllMotorPowers(power);
                break;
            case 6: // robot will rotate right
                setTargetPositionCounts(ticks, -ticks, ticks, -ticks);
                setAllMotorPowers(power);
                break;
        }

        setRTP();

        while (leftWheel.isBusy() && rightWheel.isBusy() && backLeftWheel.isBusy() && backRightWheel.isBusy()) {

        }
        resetEncoders();
        leftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void resetAngle() { //resetting the angles (after we finish turn)
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        currAngle = 0;
    }

    public double getAngle() {
        Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double changeInAngle = orientation.firstAngle - lastAngles.firstAngle; //change in angle from previous angle to current angle

        if (changeInAngle > 180) {
            changeInAngle -= 360;
        } else if (changeInAngle <= -180) {
            changeInAngle += 360;
        }
        //these if statements accommodate for the IMU only going until 180 degrees.

        currAngle += changeInAngle;
        lastAngles = orientation;

        telemetry.addData("gyro", orientation.firstAngle);
        telemetry.update();
        return currAngle;
    }

    public void turn(double degrees) {
        setRWE();
        resetAngle();
        double error = degrees;

        while (IsStopRequested == false && Math.abs(error) > 2) {
            double motorPower = (error < 0 ? -0.3 : 0.3);
            setMotorPowers(-motorPower, motorPower, -motorPower, motorPower);
            error = degrees - getAngle();

            telemetry.addData("error", error);
            telemetry.addData("angle", currAngle);
            telemetry.addData("1 imu heading", lastAngles.firstAngle);
            telemetry.addData("2 global heading", currAngle);
            telemetry.addData("3 correction", error);
            telemetry.update();


        }
        setAllMotorPowers(0);
    }

    public void forward(double degrees, double fl, double fr, double bl, double br) {

        resetAngle();
        double error = degrees;
        runtime.startTime();

        while (Math.abs(error) > 2) {
            double motorPower = (error < 0 ? -0.3 : 0.3);
            setMotorPowers(fl, fr, bl, br);
            error = degrees - getAngle();

            telemetry.addData("error", error);
            telemetry.addData("angle", currAngle);
            telemetry.addData("1 imu heading", lastAngles.firstAngle);
            telemetry.addData("2 global heading", currAngle);
            telemetry.addData("3 correction", error);
            telemetry.update();


        }
        setAllMotorPowers(0);
    }

    public void setAllMotorPowers(double power) {
        leftWheel.setPower(power);
        rightWheel.setPower(power);
        backLeftWheel.setPower(power);
        backRightWheel.setPower(power);
    }

    public void setMotorPowers(double lw, double rw, double bl, double br) {
        leftWheel.setPower(lw);
        rightWheel.setPower(rw);
        backLeftWheel.setPower(bl);
        backRightWheel.setPower(br);
    }

    public void setTargetPositionCounts(double fl_count, double fr_count, double bl_count, double br_count) {
        leftWheel.setTargetPosition((int) fl_count);
        rightWheel.setTargetPosition((int) fr_count);
        backLeftWheel.setTargetPosition((int) bl_count);
        backRightWheel.setTargetPosition((int) br_count);
    }

    public void resetEncoders() {
        leftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setRTP() {
        leftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setRUE() {
        leftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setZeroPowerBehaiv() {
        leftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void setRWE() {
        leftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void sleep(int milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}