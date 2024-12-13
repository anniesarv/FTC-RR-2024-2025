package org.firstinspires.ftc.teamcode.testing.OPS;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.teamcode.commands.drivetrain.FieldCentricCommand;
import org.firstinspires.ftc.teamcode.commands.elevator.MoveElevatorCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;

@TeleOp
public class driverone extends LinearOpMode {

    private DriveSubsystem driveSubsystem;
    private FieldCentricCommand driveCommand;
    private ElevatorSubsystem elevator;
    private MoveElevatorCommand moveElevatorCommand;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize DriveSubsystem
        driveSubsystem = new DriveSubsystem(hardwareMap);
        driveCommand = new FieldCentricCommand(driveSubsystem, gamepad1);

        // Initialize ElevatorSubsystem
        DcMotorEx motorElevator = hardwareMap.get(DcMotorEx.class, "motorElevator");
        elevator = new ElevatorSubsystem(motorElevator);
        moveElevatorCommand = new MoveElevatorCommand(elevator, 800);

        // Setup telemetry for both subsystems
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addLine("IMU and Elevator Initialized. Waiting for start...");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // Execute Drive Command
            driveCommand.execute();

            // Control the elevator
            if (gamepad1.right_bumper) {
                CommandScheduler.getInstance().schedule(new MoveElevatorCommand(elevator, 800));
            } else if (gamepad1.dpad_up) {
                elevator.setTarget(elevator.getTarget() + 20);
            } else if (gamepad1.dpad_down) {
                elevator.setTarget(elevator.getTarget() - 20);
            } else if (gamepad1.x) {
                elevator.resetEncoder();
            } else if (gamepad1.b) {
                CommandScheduler.getInstance().schedule(new MoveElevatorCommand(elevator, 600));
            }

            // Move elevator according to target
            elevator.moveElevator();

            // Telemetry for both drive and elevator
            telemetry.addData("Heading (rad)", driveSubsystem.getHeading());
            telemetry.addData("Joystick (x, y, rx)", "%.2f, %.2f, %.2f", gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
            telemetry.addData("Target Position", elevator.getTarget());
            telemetry.addData("Slide Position", elevator.getCurrentPosition());
            telemetry.update();

            // Run the Command Scheduler
            CommandScheduler.getInstance().run();
        }
    }
}
