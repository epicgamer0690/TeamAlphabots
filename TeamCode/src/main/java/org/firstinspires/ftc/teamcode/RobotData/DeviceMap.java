package org.firstinspires.ftc.teamcode.RobotData;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class DeviceMap {
    public DcMotor leftFront = null;
    public DcMotor rightFront = null;
    public DcMotor leftBottom = null;
    public DcMotor rightBottom = null;
    public Servo leftClaw = null;
    public DistanceSensor distanceSensor;

    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime ();

    public DeviceMap(){
    }

    public void init(HardwareMap hwMap){
        leftFront = hwMap.get(DcMotor.class,"leftFront");
        rightFront = hwMap.get(DcMotor.class,"rightFront");
        leftBottom = hwMap.get(DcMotor.class,"leftBottom");
        rightBottom = hwMap.get(DcMotor.class,"rightBottom");

        Rev2mDistanceSensor sensorTOF = (Rev2mDistanceSensor) distanceSensor; //TOF stands for time of flight

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBottom.setPower(0);
        rightBottom.setPower(0);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBottom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBottom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftClaw = hwMap.get(Servo.class, "leftClaw");
    }
}
