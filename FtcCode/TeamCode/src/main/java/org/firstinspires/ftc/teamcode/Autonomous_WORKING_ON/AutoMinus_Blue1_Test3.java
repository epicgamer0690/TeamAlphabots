

package org.firstinspires.ftc.teamcode.Autonomous_WORKING_ON;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

    @Autonomous(name="AutoMinus_Blue1_Test3", group="Training")
public class AutoMinus_Blue1_Test3 extends OpMode {

    DcMotor leftWheel;
    DcMotor rightWheel;
    DcMotor backLeftWheel;
    DcMotor backRightWheel;
    DcMotor armMotor;
    DcMotor carouselMotor;
    CRServo intakeServo;
    RevColorSensorV3 colorSensor;
        BNO055IMU imu;
        private Orientation lastAngles = new Orientation();
        private double currAngle = 0.0;
        boolean isStopRequested = false;
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
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);
        //initializing the IMU and setting the units needed

    }
    @Override
    public void loop() {
    }

    @Override
    public void start() {
        shippingHubLevel(195);
        resetEncoders();
        encoderMovement(60 , 1, 0.2);
        turn(90);
        outakeFunc();
        encoderMovement(60, 2, 0.2);





    }


    public void encoderMovement(double distance, int direction, double power) {

        leftWheel.setDirection(DcMotor.Direction.REVERSE);
        backLeftWheel.setDirection(DcMotor.Direction.REVERSE);

        setZeroPowerBehaiv();
        resetEncoders();
        setRUE();

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
    }
        public void resetAngle(){ //resetting the angles (after we finish turn)
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

        public void turn(double degrees){

            resetAngle();
            double error = degrees;

            while(isStopRequested == false && Math.abs(error) > 2){
                double motorPower = ( error < 0 ? -0.3 : 0.3);
                setMotorPowers(-motorPower, motorPower, -motorPower, motorPower);
                error = degrees - getAngle();

                telemetry.addData("error", error);
                telemetry.update();


            }
            setAllMotorPowers(0);
        }
        public void turnTo(double degrees) {
            Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double error = degrees - orientation.firstAngle;

            if(error > 180) {
                error -= 360;
            } else if(error < -180) {
                error += 360;
            }

            turn(error);

        }

        public void shippingHubLevel(int rotation) {
        resetEncoders();
        armMotor.setTargetPosition(rotation);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(1);
    }
    public void sleep(int milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
    public void outakeFunc(){
        intakeServo.setPower(-1);
        sleep(3000);
    }


    public void setAllMotorPowers(double power){
        leftWheel.setPower(power);
        rightWheel.setPower(power);
        backLeftWheel.setPower(power);
        backRightWheel.setPower(power);
    }
    public void setMotorPowers(double lw, double rw, double bl, double br){
        leftWheel.setPower(lw);
        rightWheel.setPower(rw);
        backLeftWheel.setPower(bl);
        backRightWheel.setPower(br);
        }
    public void setTargetPositionCounts(double fl_count,double fr_count,double bl_count,double br_count){
        leftWheel.setTargetPosition((int) fl_count);
        rightWheel.setTargetPosition((int) fr_count);
        backLeftWheel.setTargetPosition((int) bl_count);
        backRightWheel.setTargetPosition((int) br_count);
    }
    public void resetEncoders(){
        leftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void setRTP(){
        leftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void setRUE(){
        leftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void setZeroPowerBehaiv(){
        leftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    }