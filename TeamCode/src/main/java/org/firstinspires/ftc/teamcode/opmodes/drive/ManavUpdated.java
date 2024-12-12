package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

@TeleOp
public class ManavUpdated extends LinearOpMode {

    // Declare RobotHardware object
    RobotHardware robot;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the RobotHardware object
        robot = new RobotHardware();
        robot.init(hardwareMap); // Initialize all hardware components

        // Telemetry message
        telemetry.addLine("Hardware initialized. Waiting for start...");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // Get joystick inputs
            double y = -gamepad1.left_stick_y; // Forward/backward
            double x = gamepad1.left_stick_x * 1.1; // Strafe (scaled for strafing inefficiency)
            double rx = gamepad1.right_stick_x; // Rotation

            // Reset IMU yaw with the "Options" button
            if (gamepad1.options) {
                robot.initializeImu();  // Directly call the initializeImu() method from RobotHardware
                telemetry.addLine("IMU Yaw Reset!");
                telemetry.update();
            }

            // Get robot heading from IMU
            double botHeading = robot.imu.getAngularOrientation().firstAngle;

            // Field-centric transformation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            // Calculate motor powers
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            // Apply motor powers
            robot.motorLF.setPower(frontLeftPower);
            robot.motorLR.setPower(backLeftPower);
            robot.motorRF.setPower(frontRightPower);
            robot.motorRR.setPower(backRightPower);

            // Telemetry for debugging
            telemetry.addData("Heading (rad)", botHeading);
            telemetry.addData("Joystick (x, y, rx)", "%.2f, %.2f, %.2f", x, y, rx);
            telemetry.addData("Motor Powers (FL, BL, FR, BR)", "%.2f, %.2f, %.2f, %.2f",
                    frontLeftPower, backLeftPower, frontRightPower, backRightPower);
            telemetry.update();
        }
    }
}
