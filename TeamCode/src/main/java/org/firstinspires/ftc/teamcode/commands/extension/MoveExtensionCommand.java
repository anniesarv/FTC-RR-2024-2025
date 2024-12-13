package org.firstinspires.ftc.teamcode.commands.extension;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ExtensionSubsystem;

public class MoveExtensionCommand extends CommandBase {
    private final ExtensionSubsystem extension;
    private final int targetPosition;

    public MoveExtensionCommand(ExtensionSubsystem extension, int targetPositionEX) {
        this.extension = extension;
        this.targetPosition = targetPositionEX;
        addRequirements(extension);
    }

    @Override
    public void initialize() {
        extension.setTarget(targetPosition);
    }

    @Override
    public void execute() {
        extension.moveElevator();
    }

    @Override
    public boolean isFinished() {
        return Math.abs(extension.getCurrentPosition() - targetPosition) < 10;
    }

    @Override
    public void end(boolean interrupted) {
        extension.setTarget(extension.getCurrentPosition());
    }
}