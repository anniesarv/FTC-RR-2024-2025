package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveSubsystem {

    // Declare motors and IMU
    private DcMotor frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
    private BNO055IMU imu;

    public DriveSubsystem(HardwareMap hardwareMap) {
        // Initialize motors
        frontLeftMotor = hardwareMap.dcMotor.get("motorLF");
        backLeftMotor = hardwareMap.dcMotor.get("motorLR");
        frontRightMotor = hardwareMap.dcMotor.get("motorRF");
        backRightMotor = hardwareMap.dcMotor.get("motorRR");

        // Reverse necessary motors (adjust based on your robot's configuration)
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Initialize IMU
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
    }

    public void drive(double x, double y, double rx) {
        // Get robot heading from IMU
        double botHeading = imu.getAngularOrientation().firstAngle;

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
        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
    }

    public void resetIMU() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
    }

    public double getHeading() {
        return imu.getAngularOrientation().firstAngle;
    }
}