package org.firstinspires.ftc.teamcode.opmodes.elevator;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.elevator.ElevatorPIDCommand;
import org.firstinspires.ftc.teamcode.notinuse.Elevator;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

@TeleOp
public class ElevatorPIDControlOpmode extends OpMode {

    private Elevator elevator;
    private ElevatorPIDCommand elevatorPIDCommand;
    private RobotHardware robotHardware;

    @Override
    public void init() {
        robotHardware = new RobotHardware();

        elevator = new Elevator(robotHardware);


        //double desiredPosition = VerticalConstants.ElevatorPositions.MID_POSITION;  // Example position
        double desiredPosition = 200;
        elevatorPIDCommand = new ElevatorPIDCommand(elevator, desiredPosition);

        CommandScheduler.getInstance().schedule(elevatorPIDCommand);
    }

    @Override
    public void loop() {
        CommandScheduler.getInstance().run();

        telemetry.addData("Elevator Position", elevator.getPosition());
        telemetry.addData("At Bottom", elevator.atBottom());
        telemetry.update();
    }

    @Override
    public void stop() {
        CommandScheduler.getInstance().cancelAll();
    }
}
