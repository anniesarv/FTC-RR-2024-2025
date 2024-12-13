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
        double y = -gamepad.left_stick_y;
        double x = gamepad.left_stick_x * 1.1;
        double rx = gamepad.right_stick_x;

        if (gamepad.options) {
            driveSubsystem.resetIMU();
        }

        driveSubsystem.drive(x, y, rx);
    }
}




