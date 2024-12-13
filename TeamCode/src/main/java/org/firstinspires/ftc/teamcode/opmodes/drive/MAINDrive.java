package org.firstinspires.ftc.teamcode.opmodes.drive;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commands.drivetrain.FieldCentricCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;


@TeleOp(name = "Unified Field-Centric TeleOp", group = "Linear Opmode")
public class MAINDrive extends LinearOpMode {

    private DriveSubsystem driveSubsystem;
    private FieldCentricCommand driveCommand;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the drive subsystem
        driveSubsystem = new DriveSubsystem(hardwareMap);

        // Initialize the command
        driveCommand = new FieldCentricCommand(driveSubsystem, gamepad1);

        telemetry.addLine("IMU Initialized. Waiting for start...");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // Execute the drive command
            driveCommand.execute();

            // Telemetry for debugging
            telemetry.addData("Heading (rad)", driveSubsystem.getHeading());
            telemetry.addData("Joystick (x, y, rx)", "%.2f, %.2f, %.2f", gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
            telemetry.update();
        }
    }
}