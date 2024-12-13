package org.firstinspires.ftc.teamcode.opmodes.elevator;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmodes.elevator.ElevatorControl;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

@TeleOp
public class ElevatorControlOpMode extends LinearOpMode {

    private RobotHardware robotHardware = new RobotHardware();
    private ElevatorControl elevatorControl;

    @Override
    public void runOpMode() {
        // Initialize hardware
        robotHardware.init(hardwareMap);
        elevatorControl = new ElevatorControl(robotHardware);

        // Wait for the start button to be pressed
        waitForStart();

        // Example: Move elevator to position 1000
        elevatorControl.setTargetPosition(400);

        // Loop while the OpMode is active
        while (opModeIsActive() && !elevatorControl.isAtTarget()) {
            elevatorControl.update(); // Update the PID control to adjust the motor
            telemetry.addData("Elevator Position", robotHardware.motorElevator.getCurrentPosition());
            telemetry.update();
        }

        // Stop the elevator motor once the target position is reached
        elevatorControl.stop();
    }
}
