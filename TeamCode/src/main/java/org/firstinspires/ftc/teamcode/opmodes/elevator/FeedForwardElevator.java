package org.firstinspires.ftc.teamcode.opmodes.elevator;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class FeedForward extends OpMode {

    // Subsystems and hardware components

    private DcMotor motorElevator;

    private PIDController controller;
    private ElapsedTime timer = new ElapsedTime();

    public static double p = 0.005, i = 0, d = 0.00001;
    public static double kg = 0.1; // Gravity compensation
    public static int target = 1000; // Linear slide target position in encoder ticks
    public static int startPos = 0;   // Start position in encoder ticks
    public static int maxPos = 880;


    @Override
    public void init() {
        // Initialize drivebase motors

        target = 0;
        // Initialize linear slides
        motorElevator = hardwareMap.get(DcMotorEx.class, "motorElevator");
        motorElevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorElevator.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorElevator.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorElevator.setDirection(DcMotorEx.Direction.FORWARD);

        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        // Drivebase control (gamepad1)


        // Linear slides control (gamepad1)
        controller.setPID(p, i, d);
        int slidesPos = motorElevator.getCurrentPosition(); // Average position if needed

        if (gamepad1.right_bumper) {
            target = 800;
        }

        if (gamepad1.dpad_up) {
            target += 7;
        }

        else if  (gamepad1.dpad_down) {
            target -= 7;
        }

        target = Math.max(-100, Math.min(target, 1000));

        if (gamepad1.x) {
            motorElevator.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            target = 0; // Reset target to match the new encoder state
        }
        if (gamepad1.b) {
            target = 600;
        }


        double pid = controller.calculate(slidesPos, target);
        double power = pid + kg;
        motorElevator.setPower(power);






        telemetry.addData("Target Position", target);
        telemetry.addData("Slide Position ", slidesPos);
        telemetry.addData("PID Output", pid);
        telemetry.addData("Motor Power", power);
        telemetry.update();
    }
}
