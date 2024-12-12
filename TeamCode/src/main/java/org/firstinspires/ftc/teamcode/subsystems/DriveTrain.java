package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

public class DriveTrain extends SubsystemBase {
    private final DcMotor motorLF, motorRF, motorLR, motorRR;
    private final RobotHardware robotHardware; // Add RobotHardware to access IMU

    public DriveTrain(RobotHardware hardware) {
        this.robotHardware = hardware;
        this.motorLF = hardware.motorLF;
        this.motorRF = hardware.motorRF;
        this.motorLR = hardware.motorLR;
        this.motorRR = hardware.motorRR;
    }

    public void drive(double frontLeftPower, double backLeftPower, double frontRightPower, double backRightPower) {
        motorLF.setPower(frontLeftPower);
        motorLR.setPower(backLeftPower);
        motorRF.setPower(frontRightPower);
        motorRR.setPower(backRightPower);
    }

    public void stop() {
        drive(0, 0, 0, 0);
    }

    // Access the IMU from RobotHardware
    public double getBotHeading() {
        return robotHardware.imu.getAngularOrientation().firstAngle;
    }
}
