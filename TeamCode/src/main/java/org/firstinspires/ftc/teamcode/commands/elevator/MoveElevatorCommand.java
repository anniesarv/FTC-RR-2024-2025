package org.firstinspires.ftc.teamcode.testing.elevator;

import com.arcrobotics.ftclib.command.CommandBase;

public class MoveElevatorCommand extends CommandBase {
    private final ElevatorSubsystem elevator;
    private final int targetPosition;

    public MoveElevatorCommand(ElevatorSubsystem elevator, int targetPositionEl) {
        this.elevator = elevator;
        this.targetPosition = targetPositionEl;
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
