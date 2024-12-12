package org.firstinspires.ftc.teamcode.commands.drivetrain;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import java.util.function.DoubleSupplier;

public class DriveCommand extends CommandBase {
    private final DriveTrain drivetrain;
    private final DoubleSupplier leftX, leftY, rightX;  // Inputs for joystick values
    private double botHeading;

    public DriveCommand(DriveTrain drivetrain, DoubleSupplier leftX, DoubleSupplier leftY, DoubleSupplier rightX) {
        this.drivetrain = drivetrain;
        this.leftX = leftX;
        this.leftY = leftY;
        this.rightX = rightX;
        addRequirements(drivetrain); // Ensures no other commands use the drivetrain simultaneously
    }

    @Override
    public void execute() {
        // Get joystick inputs
        double y = -leftY.getAsDouble();  // Forward/backward
        double x = leftX.getAsDouble() * 1.1;  // Strafe (scaled for strafing inefficiency)
        double rx = rightX.getAsDouble();  // Rotation

        // Get robot heading from IMU
        botHeading = drivetrain.getBotHeading();

        // Field-centric transformation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        // Calculate motor powers
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        // Apply motor powers
        drivetrain.drive(frontLeftPower, backLeftPower, frontRightPower, backRightPower);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop(); // Stop motors when the command ends
    }
}
