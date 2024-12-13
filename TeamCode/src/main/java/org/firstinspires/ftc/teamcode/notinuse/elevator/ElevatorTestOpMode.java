/*package org.firstinspires.ftc.teamcode.opmodes.elevator;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.arcrobotics.ftclib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.notinuse.elevator.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.notinuse.elevator.ElevatorTestCommand;
import org.firstinspires.ftc.teamcode.notinuse.Elevatortest;

@TeleOp(name = "Elevator OpMode")
public class ElevatorTestOpMode extends OpMode {

    private Elevatortest elevatorSubsystem;
    private CommandScheduler commandScheduler;

    @Override
    public void init() {
        // Initialize the hardware for the elevator motor
        DcMotorEx motorElevator = hardwareMap.get(DcMotorEx.class, "elevatorMotor");

        // Create the elevator subsystem
        elevatorSubsystem = new Elevatortest(motorElevator);

        // Get the command scheduler instance
        commandScheduler = CommandScheduler.getInstance();

        // Initialize telemetry
        telemetry.addLine("Elevator OpMode initialized.");
    }

    @Override
    public void loop() {
        // Scheduler handles subsystem periodic updates and command execution
        commandScheduler.run();

        // Example: Assign commands based on gamepad input
        if (gamepad1.dpad_up) {
            // Move elevator to maximum position
            commandScheduler.schedule(new ElevatorTestCommand(elevatorSubsystem, Elevatortest.maxPos));
        } else if (gamepad1.dpad_down) {
            // Move elevator to minimum position
            commandScheduler.schedule(new ElevatorCommand(elevatorSubsystem, 0));
        }

        // Display current position and target on telemetry
        telemetry.addData("Current Position", elevatorSubsystem.getCurrentPosition());
        telemetry.addData("Target Position", elevatorSubsystem.getTargetPosition());
        telemetry.update();
    }

    @Override
    public void stop() {
        // Cancel all active commands when the OpMode stops
        commandScheduler.cancelAll();
    }
}*/