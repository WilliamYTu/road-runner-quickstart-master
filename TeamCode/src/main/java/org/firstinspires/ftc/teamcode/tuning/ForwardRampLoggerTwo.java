package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "TankDriveForwardRampLogger", group = "Tuning")
public class ForwardRampLoggerTwo extends LinearOpMode {

    private DcMotorEx leftMotor, rightMotor;
    private static final double MAX_POWER = 1.0; // Maximum motor power
    private static final double RAMP_DURATION = 5.0; // Duration to ramp from 0 to MAX_POWER in seconds

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize motors
        leftMotor = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightMotor = hardwareMap.get(DcMotorEx.class, "rightFront");

        // Set motor directions if necessary
        leftMotor.setDirection(DcMotorEx.Direction.REVERSE);
        rightMotor.setDirection(DcMotorEx.Direction.FORWARD);

        // Set motors to run without encoders
        leftMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize dashboard for telemetry
        FtcDashboard dashboard = FtcDashboard.getInstance();

        waitForStart();

        if (isStopRequested()) return;

        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        while (opModeIsActive() && timer.seconds() < RAMP_DURATION) {
            // Calculate the current power based on elapsed time
            double elapsedTime = timer.seconds();
            double power = (elapsedTime / RAMP_DURATION) * MAX_POWER;

            // Apply power to both motors
            leftMotor.setPower(power);
            rightMotor.setPower(power);

            // Log data to telemetry
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Elapsed Time (s)", elapsedTime);
            packet.put("Motor Power", power);
            packet.put("Left Motor Velocity", leftMotor.getVelocity());
            packet.put("Right Motor Velocity", rightMotor.getVelocity());
            dashboard.sendTelemetryPacket(packet);

            telemetry.addData("Elapsed Time (s)", elapsedTime);
            telemetry.addData("Motor Power", power);
            telemetry.addData("Left Motor Velocity", leftMotor.getVelocity());
            telemetry.addData("Right Motor Velocity", rightMotor.getVelocity());
            telemetry.update();
        }

        // Stop motors after ramping
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }
}
