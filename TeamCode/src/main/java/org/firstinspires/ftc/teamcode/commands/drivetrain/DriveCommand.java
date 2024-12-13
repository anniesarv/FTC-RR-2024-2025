package org.firstinspires.ftc.teamcode.commands.drivetrain;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.notinuse.DriveTrain;
import java.util.function.DoubleSupplier;

public class DriveCommand extends CommandBase {
    private final DriveTrain drivetrain;
    private final DoubleSupplier leftX, leftY, rightX;
    private double botHeading;

    public DriveCommand(DriveTrain drivetrain, DoubleSupplier leftX, DoubleSupplier leftY, DoubleSupplier rightX) {
        this.drivetrain = drivetrain;
        this.leftX = leftX;
        this.leftY = leftY;
        this.rightX = rightX;
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        // Get joystick inputs
        double y = -leftY.getAsDouble();
        double x = leftX.getAsDouble() * 1.1;
        double rx = rightX.getAsDouble();

        botHeading = drivetrain.getBotHeading();

        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        drivetrain.drive(frontLeftPower, backLeftPower, frontRightPower, backRightPower);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop(); // Stop motors when the command ends
    }
}
