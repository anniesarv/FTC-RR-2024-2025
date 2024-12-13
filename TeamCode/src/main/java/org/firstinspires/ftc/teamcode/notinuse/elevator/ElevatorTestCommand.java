package org.firstinspires.ftc.teamcode.notinuse.elevator;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.command.SubsystemBase;

public class ElevatorTestCommand extends SubsystemBase {

    private DcMotorEx motorElevator;
    private PIDController controller;

    public static double p = 0.005, i = 0, d = 0.00001;
    public static double kg = 0.1; // Gravity compensation
    public static int maxPos = 880;
    private int target = 0;

    public ElevatorTestCommand(DcMotorEx motorElevator) {
        this.motorElevator = motorElevator;
        this.motorElevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        this.motorElevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motorElevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.motorElevator.setDirection(DcMotorEx.Direction.FORWARD);

        this.controller = new PIDController(p, i, d);
    }

    public void setTargetPosition(int target) {
        this.target = Math.max(0, Math.min(target, maxPos));
    }

    public void resetEncoder() {
        motorElevator.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorElevator.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        target = 0;
    }

    public void periodic() {
        update();
    }

    private void update() {
        int slidesPos = motorElevator.getCurrentPosition();
        controller.setPID(p, i, d);
        double pid = controller.calculate(slidesPos, target);
        double power = pid + kg;
        motorElevator.setPower(power);
    }

    public int getCurrentPosition() {
        return motorElevator.getCurrentPosition();
    }

    public int getTargetPosition() {
        return target;
    }
}
