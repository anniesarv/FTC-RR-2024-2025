package org.firstinspires.ftc.teamcode.commands.elevator;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;

public class MoveElevatorCommand extends CommandBase {
    private final ElevatorSubsystem elevator;
    private final int targetPosition;

    public MoveElevatorCommand(ElevatorSubsystem elevator, int targetPosition) {
        this.elevator = elevator;
        this.targetPosition = targetPosition;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        elevator.setTarget(targetPosition);
    }

    @Override
    public void execute() {
        elevator.moveElevator();
    }

    @Override
    public boolean isFinished() {
        return Math.abs(elevator.getCurrentPosition() - targetPosition) < 10;
    }

    @Override
    public void end(boolean interrupted) {
        elevator.setTarget(elevator.getCurrentPosition());
    }
}
