package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.arcrobotics.ftclib.controller.PIDController;

// Subsystem: Elevator
public class ElevatorSubsystem extends SubsystemBase {
    private final DcMotorEx motorElevator;
    private final PIDController controller;

    public static double p = 0.01, i = 0, d = 0.00001;
    public static double kg = 0.1; // Gravity compensation
    private int target = 0; // Linear slide target position

    public ElevatorSubsystem(DcMotorEx motor) {
        motorElevator = motor;
        motorElevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorElevator.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorElevator.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorElevator.setDirection(DcMotorEx.Direction.FORWARD);

        controller = new PIDController(p, i, d);
    }

    public void resetEncoder() {
        motorElevator.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorElevator.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        target = 0; // Reset the target position to match the new encoder state
    }

    public void setTarget(int newTarget) {
        target = Math.max(-100, Math.min(newTarget, 1000)); // Clamp target to limits
    }

    public void moveElevator() {
        int slidesPos = motorElevator.getCurrentPosition();
        double pid = controller.calculate(slidesPos, target);
        double power = pid + kg;
        motorElevator.setPower(power);
    }

    public int getCurrentPosition() {
        return motorElevator.getCurrentPosition();
    }

    public int getTarget() {
        return target;
    }
}

