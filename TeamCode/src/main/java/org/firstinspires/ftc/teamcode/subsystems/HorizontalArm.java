package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;



public class HorizontalArm extends SubsystemBase {

    private final DcMotor motorHA;
    private final DigitalChannel limitSwitchHA;
    private final PIDController pidController;


    private static final double kP = 1.0;
    private static final double kI = 0.0;
    private static final double kD = 0.0;




    public HorizontalArm(RobotHardware robotHardware) {
        this.motorHA = robotHardware.motorVA;
        this.limitSwitchHA = robotHardware.limitSwitchVA;

        pidController = new PIDController(kP, kI, kD);

    }

    public void moveToPosition(int targetPosition) {
        double currentPosition = motorHA.getCurrentPosition();
        double power = pidController.calculate(currentPosition, targetPosition);
        motorHA.setPower(power);
    }

    public void moveArm(double power) {
        motorHA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorHA.setPower(power);
    }

    public void stopArm() {
        motorHA.setPower(0);
    }

    public void checkAndResetEncoder() {
        if (!limitSwitchHA.getState()) {
            motorHA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorHA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    @Override
    public void periodic() {
        checkAndResetEncoder();
    }
}
