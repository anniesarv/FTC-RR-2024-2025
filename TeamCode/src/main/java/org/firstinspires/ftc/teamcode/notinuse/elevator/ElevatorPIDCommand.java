package org.firstinspires.ftc.teamcode.commands.elevator;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import org.firstinspires.ftc.teamcode.notinuse.VerticalConstants;
import org.firstinspires.ftc.teamcode.notinuse.Elevator;

public class ElevatorPIDCommand extends CommandBase {

    private Elevator elevator;
    private double setPoint;
    private PIDFController controller;

    public ElevatorPIDCommand(Elevator elevator, double setPoint) {

        this.elevator = elevator;
        this.setPoint = setPoint;
        VerticalConstants.ElevatorCoefficients c = new VerticalConstants.ElevatorCoefficients();
        controller = new PIDFController(c.KP, c.KI, c.KD, c.KF);
        controller.setSetPoint(setPoint);
        controller.setTolerance(VerticalConstants.ElevatorConstants.POSITION_TOLERANCE, VerticalConstants.ElevatorConstants.VELOCITY_TOLERANCE);
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        controller.calculate(elevator.getPosition(), setPoint);
    }

    @Override
    public void execute() {
        double goingDown = (setPoint <= VerticalConstants.ElevatorPositions.LOWER_LIMIT) ? -VerticalConstants.ElevatorCoefficients.KG : 0.0;
        elevator.setSpeed(controller.calculate(elevator.getPosition(), setPoint) + goingDown);
    }

    @Override
    public boolean isFinished() {
        return (setPoint <= VerticalConstants.ElevatorPositions.LOWER_LIMIT) ? elevator.atBottom() : controller.atSetPoint();
    }

    @Override
    public void end(boolean interrupted) {
        elevator.setSpeed(0.0);
    }
}
