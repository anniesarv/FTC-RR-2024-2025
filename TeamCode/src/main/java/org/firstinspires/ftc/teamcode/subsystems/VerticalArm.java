package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;



public class VerticalArm extends SubsystemBase {

    private final DcMotor motorVA;
    private final DigitalChannel limitSwitchVA;
    private final PIDFController pidfController;


    private static final double kP = 1.0;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double kF = 0.0;



    public VerticalArm(RobotHardware robotHardware) {
        this.motorVA = robotHardware.motorVA;
        this.limitSwitchVA = robotHardware.limitSwitchVA;

        pidfController = new PIDFController(kP, kI, kD, kF);

    }

    public void moveToPosition(int targetPosition) {
        double currentPosition = motorVA.getCurrentPosition();
        double power = pidfController.calculate(currentPosition, targetPosition);
        motorVA.setPower(power);
    }

    public void moveArm(double power) {
        motorVA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorVA.setPower(power);
    }

    public void stopArm() {
        motorVA.setPower(0);
    }

    public void checkAndResetEncoder() {
        if (!limitSwitchVA.getState()) {
            motorVA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorVA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    @Override
    public void periodic() {
        checkAndResetEncoder();
    }
}
