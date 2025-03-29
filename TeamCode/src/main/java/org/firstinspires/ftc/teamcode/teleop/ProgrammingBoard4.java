package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ProgrammingBoard4 {
    private DigitalChannel touchSensor;
    private DcMotor motor1;
    private DcMotor motor2;
    private DcMotor armMotor;
    private DcMotor intakeMotor;
    private CRServo intakeServo;
    private Servo clawServo;
    private double ticksPerRotation;


    public void init(HardwareMap hwMap){
        motor2 = hwMap.get(DcMotor.class, "leftFront");
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor1 = hwMap.get(DcMotor.class, "rightFront");
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor = hwMap.get(DcMotor.class, "armMotor");
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor = hwMap.get(DcMotor.class, "intakeMotor");
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeServo = hwMap.get(CRServo.class, "intakeServo");
        clawServo = hwMap.get(Servo.class, "clawServo");
        clawServo.scaleRange(0.5, 1);
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        ticksPerRotation = motor1.getMotorType().getTicksPerRev();
    }
    public boolean isTouchSensorPressed(){
        return !touchSensor.getState();
    }
    public int getMotorPosition(String motorName){
        if (motorName == "motor1"){
            return motor1.getCurrentPosition();
        } else if (motorName == "motor2") {
            return motor2.getCurrentPosition();
        } else if (motorName == "armMotor"){
            return armMotor.getCurrentPosition();
        } else if (motorName == "intakeMotor"){
            return intakeMotor.getCurrentPosition();
        } else {
            return 0;
        }
    }
    public void setMotorSpeed(double speed){
        motor1.setPower(speed);
        motor2.setPower(-speed);
    }
    public void moveIntakeServo(float direction){
        intakeServo.setPower(direction / 2);
    }
    public void moveClawServo(double position){
        clawServo.setPosition(position);
    }
    public void moveClaw(double position){
        clawServo.setPosition(position);
    }
    public void moveArm(double direction){
        armMotor.setPower(direction / 2);
    }
    public void turnRobot(double speed){
        motor1.setPower(speed);
        motor2.setPower(speed);
    }
    public void moveIntake(float direction){
        intakeMotor.setPower(direction / 2);
    }
    public void resetEncoders(){
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }
    public double getMotorRotations(String motorName) {
        if (motorName == "motor1"){
            return motor1.getCurrentPosition();
        } else if (motorName == "motor2") {
            return motor2.getCurrentPosition();
        } else if (motorName == "armMotor"){
            return armMotor.getCurrentPosition();
        } else if (motorName == "intakeMotor"){
            return intakeMotor.getCurrentPosition();
        } else {
            return 0;
        }
    }
    public void setZeroPowerBehavior(boolean gamepad1a, boolean gamepad1b){
        if (gamepad1a == true){
            motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        if (gamepad1b == true){
            motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }
}