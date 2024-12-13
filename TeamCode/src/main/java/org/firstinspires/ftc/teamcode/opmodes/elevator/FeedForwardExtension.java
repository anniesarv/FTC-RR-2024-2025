package org.firstinspires.ftc.teamcode.opmodes.elevator;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class FeedForwardExtension extends OpMode {

    // Subsystems and hardware components

    private DcMotor motorExtension;

    private PIDController controller2;
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
        motorExtension = hardwareMap.get(DcMotorEx.class, "motorExtension");
        motorExtension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorExtension.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorExtension.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorExtension.setDirection(DcMotorEx.Direction.FORWARD);


        controller2 = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        // Drivebase control (gamepad1)


        // Linear slides control (gamepad1)
        controller2.setPID(p, i, d);
        int slidesPos = motorExtension.getCurrentPosition(); // Average position if needed

        if (gamepad1.left_bumper) {
            target = 800;
        }

        if (gamepad1.dpad_left) {
            target += 7;
        }

        else if  (gamepad1.dpad_right) {
            target -= 7;
        }

        target = Math.max(-100, Math.min(target, 1000));

        if (gamepad1.x) {
            motorExtension.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            target = 0; // Reset target to match the new encoder state
        }
        if (gamepad1.b) {
            target = 600;
        }


        double pid = controller2.calculate(slidesPos, target);
        double power = pid + kg;
        motorExtension.setPower(power);






        telemetry.addData("Target Position", target);
        telemetry.addData("Slide Position ", slidesPos);
        telemetry.addData("PID Output", pid);
        telemetry.addData("Motor Power", power);
        telemetry.update();
    }
}
