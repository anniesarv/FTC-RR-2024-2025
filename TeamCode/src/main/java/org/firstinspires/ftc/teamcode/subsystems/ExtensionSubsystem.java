package org.firstinspires.ftc.teamcode.testing.extension;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.arcrobotics.ftclib.controller.PIDController;

// Subsystem: Elevator
public class ExtensionSubsystem extends SubsystemBase {
    private final DcMotorEx motorExtension;
    private final PIDController controller2;

    public static double p = 0.01, i = 0, d = 0.00001;
    public static double kg = 0.1; // Gravity compensation
    private int target = 0; // Linear slide target position

    public ExtensionSubsystem(DcMotorEx motor) {
        motorExtension = motor;
        motorExtension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorExtension.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorExtension.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorExtension.setDirection(DcMotorEx.Direction.FORWARD);

        controller2 = new PIDController(p, i, d);
    }

    public void resetEncoder() {
        motorExtension.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorExtension.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        target = 0; // Reset the target position to match the new encoder state
    }

    public void setTarget(int newTarget) {
        target = Math.max(-100, Math.min(newTarget, 1000)); // Clamp target to limits
    }

    public void moveElevator() {
        int slidesPos = motorExtension.getCurrentPosition();
        double pid = controller2.calculate(slidesPos, target);
        double power = pid + kg;
        motorExtension.setPower(power);
    }

    public int getCurrentPosition() {
        return motorExtension.getCurrentPosition();
    }

    public int getTarget() {
        return target;
    }
}

