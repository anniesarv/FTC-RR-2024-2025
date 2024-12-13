package org.firstinspires.ftc.teamcode.testing;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.teamcode.notinuse.elevator.OtherElevatorSubsystem;

@TeleOp
public class FeedForwardElevatorOpMode extends OpMode {
    private ElevatorSubsystem elevator;
    private MoveElevatorCommand moveElevatorCommand;

    @Override
    public void init() {
        DcMotorEx motorElevator = hardwareMap.get(DcMotorEx.class, "motorElevator");
        elevator = new ElevatorSubsystem(motorElevator);

        moveElevatorCommand = new MoveElevatorCommand(elevator, 800);

        CommandScheduler.getInstance().reset();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        if (gamepad1.right_bumper) {
            CommandScheduler.getInstance().schedule(
                    new MoveElevatorCommand(elevator, 800)
            );
        } else if (gamepad1.dpad_up) {
            elevator.setTarget(elevator.getTarget() + 7);
        } else if (gamepad1.dpad_down) {
            elevator.setTarget(elevator.getTarget() - 7);
        } else if (gamepad1.x) {
            elevator.resetEncoder();
        } else if (gamepad1.b) {
            CommandScheduler.getInstance().schedule(new MoveElevatorCommand(elevator, 600));
        }

        elevator.moveElevator();

        telemetry.addData("Target Position", elevator.getTarget());
        telemetry.addData("Slide Position", elevator.getCurrentPosition());
        telemetry.update();

        CommandScheduler.getInstance().run();
    }
}