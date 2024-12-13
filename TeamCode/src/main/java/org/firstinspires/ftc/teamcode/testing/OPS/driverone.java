package org.firstinspires.ftc.teamcode.testing.OPS;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.commands.drivetrain.FieldCentricCommand;
import org.firstinspires.ftc.teamcode.commands.elevator.MoveElevatorCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;

@TeleOp
public class driveone extends OpMode {

    // Declare subsystems and commands
    private DriveSubsystem driveSubsystem;
    private FieldCentricCommand driveCommand;

    private ElevatorSubsystem elevator;

    @Override
    public void init() {
        // Initialize DriveSubsystem
        driveSubsystem = new DriveSubsystem(hardwareMap);
        // Initialize FieldCentricCommand for drive
        driveCommand = new FieldCentricCommand(driveSubsystem, gamepad1);

        // Initialize ElevatorSubsystem
        DcMotorEx elevatorMotor = hardwareMap.get(DcMotorEx.class, "motorElevator");
        elevator = new ElevatorSubsystem(elevatorMotor);

        CommandScheduler.getInstance().reset();

        telemetry.addLine("IMU Initialized. Waiting for start...");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Execute drive command (field-centric control with gamepad1)
        driveCommand.execute();

        // **Elevator controls** using gamepad1 d-pad
        if (gamepad1.dpad_down) {
            // Move elevator up
            int newTarget = elevator.getTarget() + 7;  // Adjust this value based on your requirements
            MoveElevatorCommand moveUpCommand = new MoveElevatorCommand(elevator, newTarget);
            CommandScheduler.getInstance().schedule(moveUpCommand);
        } else if (gamepad1.dpad_up) {
            // Move elevator down
            int newTarget = elevator.getTarget() - 7;  // Adjust this value based on your requirements
            MoveElevatorCommand moveDownCommand = new MoveElevatorCommand(elevator, newTarget);
            CommandScheduler.getInstance().schedule(moveDownCommand);
        }

        // Reset elevator encoder with the 'X' button
        if (gamepad1.x) {
            elevator.resetEncoder();
        }

        // Telemetry for debugging
        telemetry.addData("Heading (rad)", driveSubsystem.getHeading());
        telemetry.addData("Joystick (x, y, rx)", "%.2f, %.2f, %.2f", gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
        telemetry.addData("Elevator Target", elevator.getTarget());
        telemetry.addData("Elevator Position", elevator.getCurrentPosition());
        telemetry.update();

        // Run the command scheduler to execute any scheduled commands
        CommandScheduler.getInstance().run();
    }
}
