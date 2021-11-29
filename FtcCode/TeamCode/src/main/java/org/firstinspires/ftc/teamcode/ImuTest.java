package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
//importing all the needed functions

@Autonomous(name="ImuTest", group="Training")
public class ImuTest extends OpMode{

    DcMotor leftWheel;
    DcMotor rightWheel;
    DcMotor backLeftWheel;
    DcMotor backRightWheel;
    BNO055IMU imu;
    private Orientation lastAngles = new Orientation();
    private double currAngle = 0.0;
    boolean isStopRequested = false;
    //initializing each motor and some important variable.

@Override
public void init() {

    leftWheel = hardwareMap.dcMotor.get("left_wheel");
    rightWheel = hardwareMap.dcMotor.get("right_wheel");
    backRightWheel = hardwareMap.dcMotor.get("back_right_wheel");
    backLeftWheel = hardwareMap.dcMotor.get("back_left_wheel");
    imu = hardwareMap.get(BNO055IMU.class, "imu");
    //Defining Hardware Map

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
public void start() { //when the start button is pressed on the driver hub, this code runs

    turn(90); //turning robot 90 degrees counter-clockwise


}
@Override
public void loop(){ //items in this loop run infinitely until the program ends

}

@Override
public void stop(){ //items in this method run once the stop button is pressed.
    isStopRequested = true;
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
        setMotorPower(-motorPower, motorPower, -motorPower, motorPower);
        error = degrees - getAngle();

        telemetry.addData("error", error);
        telemetry.update();


    }
    setAllPower(0);
}
public void sleep(int milliseconds) {
    try {
        Thread.sleep(milliseconds);
    } catch (InterruptedException e) {
        e.printStackTrace();
    }
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
public void setAllPower(double pw) { setMotorPower(pw, pw, pw, pw);
}
public void setMotorPower(double lw, double rw, double bl, double br){
    leftWheel.setPower(lw);
    rightWheel.setPower(rw);
    backLeftWheel.setPower(bl);
    backRightWheel.setPower(br);
}
}