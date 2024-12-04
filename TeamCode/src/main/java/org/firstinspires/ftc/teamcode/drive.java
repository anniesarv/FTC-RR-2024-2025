package org.firstinspires.ftc.teamcode;


import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@TeleOp
public class drive extends OpMode {

    private Motor front_left, front_right, back_left, back_right;
    private MecanumDrive mecanumDrive;
    private BNO055IMU imu;

    @Override
    public void init() {
        front_left = new Motor(hardwareMap, "motorLF");
        front_right = new Motor(hardwareMap, "motorRF");
        back_left = new Motor(hardwareMap, "motorLR");
        back_right = new Motor(hardwareMap, "motorRR");



        mecanumDrive = new MecanumDrive(front_left, front_right, back_left, back_right);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);

        telemetry.addLine("IMU Initialized");
    }

    @Override
    public void loop() {
        // Get joystick inputs
        double strafeSpeed = gamepad1.left_stick_x;  //strafe
        double forwardSpeed = -gamepad1.left_stick_y; //forqard
        double turn = gamepad1.right_stick_x;        // rotate


        double heading = imu.getAngularOrientation().firstAngle;

        // field centric
        mecanumDrive.driveFieldCentric(strafeSpeed, forwardSpeed, turn, heading);

        telemetry.addData("strafe", strafeSpeed);
        telemetry.addData("forward", forwardSpeed);
        telemetry.addData("turn", turn);
        telemetry.addData("heading", heading);
        telemetry.update();
    }
}
