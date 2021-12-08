package org.firstinspires.ftc.teamcode.Movement_Sensor_Test.sensor_test.Sensors_test;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;


/**
 * This 2020-2021 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the Freight Frenzy game elements.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Autonomous(name = "Concept: TensorFlow Object Detection", group = "Concept")
public class ObjectDetectionTesting extends LinearOpMode {
    /* Note: This sample uses the all-objects Tensor Flow model (FreightFrenzy_BCDM.tflite), which contains
     * the following 4 detectable objects
     *  0: Ball,
     *  1: Cube,
     *  2: Duck,
     *  3: Marker (duck location tape marker)
     *
     *  Two additional model assets are available which only contain a subset of the objects:
     *  FreightFrenzy_BC.tflite  0: Ball,  1: Cube
     *  FreightFrenzy_DM.tflite  0: Duck,  1: Marker
     */
    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
            "Ball",
            "Cube",
            "Duck",
            "Marker"
    };

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
    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "AceJqxr/////AAABmfhCK5bvWUGbtwq2Aklej7UU4PMgSzBzmvS8GTqRPrVjYpKWEX6XGiwzt85GlEialLkazXNjPIK9y/u4QVNGg9ZytchGG9StZFyP2Wo9GCBa32Zr9jLSDq4kNCpaxFgfV/VhkmqvdzQjPG+i3LlRB8gnK/VCrS8ofYJe907hpUg9z2vS9SnTWzrdLGA3eAOs6oAY4FUBgzze/FFV44ufBZWsNBxBC2RqwEX1m60Q07xUt20q9pf7nU+XPLkbZxp0FCsUWETjdk08M9Oh+fgJJ56cyVey7I98zPRH39b5aOze3ynMkjcW2BjkI5e2VQN6GWe17Q662qUBwJ5R/yaFV0avDp1+u5aQqN8xdl+dfVIp";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();
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
        leftWheel.setDirection(DcMotor.Direction.REVERSE);
        backLeftWheel.setDirection(DcMotor.Direction.REVERSE);
        setZeroPowerBehaiv();
        setAllMotorPowers(0);

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(2.5, 16.0/9.0);
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());

                        // step through the list of recognitions and display boundary info.
                        int i = 0;
                        boolean isDuckDetected = false;
                        boolean isCubeDetected = false;
                        boolean isBallDetected = false;
                        boolean isMarkerDetected = false;
                        for (Recognition recognition : updatedRecognitions) {
                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                    recognition.getLeft(), recognition.getTop());
                            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                    recognition.getRight(), recognition.getBottom());
                            i++;

                            // check label to see if the camera now sees a Duck         ** ADDED **
                            if (recognition.getLabel().equals("Duck")) {            //  ** ADDED **
                                isDuckDetected = true;                             //  ** ADDED **
                                telemetry.addData("Object Detected", "Duck");
                                encoderMovement(50, 1, 0.5);//  ** ADDED **
                            } else {                                               //  ** ADDED **
                                isDuckDetected = false;                            //  ** ADDED **
                            }
                            if (recognition.getLabel().equals("Cube")) {            //  ** ADDED **
                                isCubeDetected = true;                             //  ** ADDED **
                                telemetry.addData("Object Detected", "Cube");

                            } else {                                               //  ** ADDED **
                                isCubeDetected = false;                            //  ** ADDED **
                            }
                            if (recognition.getLabel().equals("Ball")) {            //  ** ADDED **
                                isBallDetected = true;                             //  ** ADDED **
                                telemetry.addData("Object Detected", "Ball");
                            } else {                                               //  ** ADDED **
                                isBallDetected = false;                            //  ** ADDED **
                            }
                            if (recognition.getLabel().equals("Marker")) {            //  ** ADDED **
                                isMarkerDetected = true;                             //  ** ADDED **
                                telemetry.addData("Object Detected", "Marker");
                            } else {                                               //  ** ADDED **
                                isMarkerDetected = false;                            //  ** ADDED **
                            }
                        }
                        telemetry.update();
                    }
                }
            }
        }

    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }


    public void encoderMovement(double distance, int direction, double power) {

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
    public void carouselFunc(){
        carouselMotor.setPower(0.5);
        sleep(2000);
        carouselMotor.setPower(0);
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

    public void turnLeft(double degrees){

        resetAngle();
        double error = degrees;

        while(opModeIsActive() && Math.abs(error) > 2){
            double motorPower = ( error < 0 ? -0.3 : 0.3);
            setMotorPowers(motorPower, -motorPower, motorPower, -motorPower);
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
    public void turnRight(double degrees){

        resetAngle();
        double error = degrees;

        while(opModeIsActive() && Math.abs(error) > 2){
            double motorPower = ( error < 0 ? -0.3 : 0.3);
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
    public void turnTo(double degrees) {
        Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double error = degrees - orientation.firstAngle;

        if(error > 180) {
            error -= 360;
        } else if(error < -180) {
            error += 360;
        }

        turnLeft(error);

    }

    public void shippingHubLevel(int rotation, double pwr) {
        resetEncoders();
        armMotor.setTargetPosition(rotation);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(pwr);
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

