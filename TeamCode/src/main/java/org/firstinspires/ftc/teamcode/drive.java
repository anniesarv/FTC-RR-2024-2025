package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@TeleOp
public class drive extends OpMode {

    private Motor front_left, front_right, back_left, back_right;
    private MecanumDrive mecanumDrive;
    private RevIMU imu; // using revIMU instead of BNO055IMU

    @Override
    public void init() {
        front_left = new Motor(hardwareMap, "motorLF");
        front_right = new Motor(hardwareMap, "motorRF");
        back_left = new Motor(hardwareMap, "motorLR");
        back_right = new Motor(hardwareMap, "motorRR");

        mecanumDrive = new MecanumDrive(front_left, front_right, back_left, back_right);

        // initialize revIMU
        imu = new RevIMU(hardwareMap, "imu");

        // set hub orientation
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT
        );




        telemetry.addLine("IMU initialized with orientation");
    }

    @Override
    public void loop() {
        // get joystick inputs
        double strafeSpeed = gamepad1.left_stick_x;  // strafe
        double forwardSpeed = -gamepad1.left_stick_y; // forward
        double turn = gamepad1.right_stick_x;        // rotate

        // Get the heading from RevIMU (IN RADIANS I HOPE)
        double heading = imu.getHeading();

        // use the heading for field-centric control
        mecanumDrive.driveFieldCentric(strafeSpeed, forwardSpeed, turn, heading);

        // debugging info
        telemetry.addData("strafe", strafeSpeed);
        telemetry.addData("forward", forwardSpeed);
        telemetry.addData("turn", turn);
        telemetry.addData("heading (radians)", heading);
        telemetry.update();
    }
}
