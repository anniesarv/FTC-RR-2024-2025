package org.firstinspires.ftc.teamcode.opmodes.drive;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

@TeleOp
public class DriveComplete extends LinearOpMode {


    private RobotHardware robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotHardware();
        robot.init(hardwareMap);

        telemetry.addLine("Hardware Initialized. Waiting for start...");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Forward/backward
            double x = gamepad1.left_stick_x * 1.1; // Strafe (
            double rx = gamepad1.right_stick_x; // Rotation

            // reset IUM yaw with the "Options" button
            if (gamepad1.options) {
                robot.initializeImu(); // Reset IMU using the RobotHardware method
                telemetry.addLine("IMU Yaw Reset!");
                telemetry.update();
            }

            double botHeading = robot.imu.getAngularOrientation().firstAngle;

            // field-centric transformation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            robot.motorLF.setPower(frontLeftPower);
            robot.motorLR.setPower(backLeftPower);
            robot.motorRF.setPower(frontRightPower);
            robot.motorRR.setPower(backRightPower);

            telemetry.addData("Heading (rad)", botHeading);
            telemetry.addData("Joystick (x, y, rx)", "%.2f, %.2f, %.2f", x, y, rx);
            telemetry.addData("Motor Powers (FL, BL, FR, BR)", "%.2f, %.2f, %.2f, %.2f",
                    frontLeftPower, backLeftPower, frontRightPower, backRightPower);
            telemetry.update();
        }
    }
}
