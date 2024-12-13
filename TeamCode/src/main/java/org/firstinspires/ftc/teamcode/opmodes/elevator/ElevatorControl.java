package org.firstinspires.ftc.teamcode.opmodes.elevator;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.arcrobotics.ftclib.controller.PIDController;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

@Config
public class ElevatorControl {

    private RobotHardware robotHardware;
    private PIDFController pidfController;

    // PIDF constants (these can be tuned)
    public static double kP = 0.5;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kF = 0.5;

    private DcMotor motorElevator;

    public ElevatorControl(RobotHardware robotHardware) {
        this.robotHardware = robotHardware;
        this.motorElevator = robotHardware.motorElevator;

        // Initialize the PIDF Controller with the constants
        pidfController = new PIDFController(kP, kI, kD, kF);

        // Set the elevator motor to run using encoders
        motorElevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setTargetPosition(int targetPosition) {
        // Set the target position for the PID loop
        pidfController.setSetPoint(targetPosition);
    }

    public void update() {
        // Get the current position of the elevator motor
        int currentPosition = motorElevator.getCurrentPosition();

        // Update the PIDF controller with the current position and get the output
        double output = pidfController.calculate(currentPosition);

        // Set the power to the elevator motor based on the PID controller output
        motorElevator.setPower(output);
    }

    public boolean isAtTarget() {
        // Check if the elevator is within a certain tolerance of the target position
        return Math.abs(motorElevator.getCurrentPosition() - pidfController.getSetPoint()) < 10;
    }

    public void stop() {
        // Stop the elevator motor
        motorElevator.setPower(0);
    }

    // Optionally, you could also add methods to reset PID values or tune them dynamically
}
