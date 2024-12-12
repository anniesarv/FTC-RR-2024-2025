package org.firstinspires.ftc.teamcode.commands.elevator;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import java.util.function.DoubleSupplier;

public class ElevatorCommand extends CommandBase {
    private final Elevator elevator;
    private final DoubleSupplier speed;

    public ElevatorCommand(Elevator elevator, DoubleSupplier speed) {
        this.elevator = elevator;
        this.speed = speed;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        elevator.setSpeed(speed.getAsDouble());
    }


    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted) {
            elevator.setSpeed(0.0);
        }
    }
}
