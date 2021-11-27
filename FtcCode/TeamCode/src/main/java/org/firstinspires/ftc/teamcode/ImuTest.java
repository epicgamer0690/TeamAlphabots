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
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="ImuTest", group="Training")
public class ImuTest extends LinearOpMode{

    DcMotor leftWheel;
    DcMotor rightWheel;
    DcMotor backLeftWheel;
    DcMotor backRightWheel;
    BNO055IMU imu;
    private Orientation lastAngles = new Orientation();
    private double currAngle = 0.0;
    private double targetAngle;
    private double kP, kI, kD;
    private double accumulatedError = 0;
    private ElapsedTime timer = new ElapsedTime();
    private double lastError = 0.0;
    private double lastTime = 0.0;


    @Override
public void runOpMode() {

    leftWheel = hardwareMap.dcMotor.get("left_wheel");
    rightWheel = hardwareMap.dcMotor.get("right_wheel");
    backRightWheel = hardwareMap.dcMotor.get("back_right_wheel");
    backLeftWheel = hardwareMap.dcMotor.get("back_left_wheel");
    imu = hardwareMap.get(BNO055IMU.class, "imu");
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
    parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
    parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
    parameters.loggingEnabled      = true;
    parameters.loggingTag          = "IMU";
    parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
    imu.initialize(parameters);


    waitForStart();

    turn(90);
    sleep(3000);
    turnTo(-90);




}
public void resetAngle(){
    lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    currAngle = 0;
}
public double getAngle(){
    Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    double changeInAngle = orientation.firstAngle - lastAngles.firstAngle;

    if(changeInAngle > 180){
        changeInAngle -= 360;
    }
    else if(changeInAngle <= -180){
        changeInAngle += 360;
    }

    currAngle += changeInAngle;
    lastAngles = orientation;
    telemetry.addData("gyro", orientation.firstAngle);
    telemetry.update();
    return currAngle;
}
public double getAbsoluteAngle() {
    return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
}
void turnToPID(double targetAngle) {
    turnPIDController(targetAngle, 0.01, 0, 0.003);
    while(opModeIsActive() && Math.abs(targetAngle - getAbsoluteAngle()) > 1) {
        double motorPower = update(getAbsoluteAngle());
        setMotorPower(-motorPower, motorPower, - motorPower, motorPower);
    }
    setAllPower(0);
}
void turnPID(double degrees) {
       turnToPID( degrees + getAbsoluteAngle());
}


public void turnPIDController(double target, double p, double i, double d) {
        targetAngle = target;

        kP = p;
        kI = i;
        kD = d;
}

public double update(double currentAngle) {
//turnPIDController
    double error = targetAngle - currentAngle;
    error %= 360;
    error += 360;
    error %= 360;

    if(error > 180) {
        error -= 360;
    }
        accumulatedError += error;
        if(Math.abs(error) < 1) {
            accumulatedError = 0;
        }
        accumulatedError = Math.abs(accumulatedError) * Math.signum(error);
        double slope = 0;
        if(lastTime > 0){
            slope = (error - lastError)/(timer.milliseconds() - lastTime);
        }
        lastTime = timer.milliseconds();
        lastError = error;

        double motorPower = 0.1 * Math.signum(error) + 0.9 *Math.tanh(kP * error + kI * accumulatedError + kD * slope);
        return motorPower;


    }







public void turn(double degrees){

    resetAngle();
    double error = degrees;

    while(opModeIsActive() && Math.abs(error) > 2){
        double motorPower = ( error < 0 ? -0.3 : 0.3);
        setMotorPower(-motorPower, motorPower, -motorPower, motorPower);
        error = degrees - getAngle();
        telemetry.addData("error", error);
        telemetry.update();


    }
    setAllPower(0);
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