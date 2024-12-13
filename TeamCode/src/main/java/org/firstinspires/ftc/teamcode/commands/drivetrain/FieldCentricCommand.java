package org.firstinspires.ftc.teamcode.commands.drivetrain;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

public class FieldCentricCommand {

    private final DriveSubsystem driveSubsystem;
    private final Gamepad gamepad;

    public FieldCentricCommand(DriveSubsystem driveSubsystem, Gamepad gamepad) {
        this.driveSubsystem = driveSubsystem;
        this.gamepad = gamepad;
    }

    public void execute() {
        // Get joystick inputs
        double y = -gamepad.left_stick_y; // Forward/backward
        double x = gamepad.left_stick_x * 1.1; // Strafe (scaled for strafing inefficiency)
        double rx = gamepad.right_stick_x; // Rotation

        // Reset IMU yaw with the "Options" button
        if (gamepad.options) {
            driveSubsystem.resetIMU();
        }

        // Execute the drive logic
        driveSubsystem.drive(x, y, rx);
    }
}




