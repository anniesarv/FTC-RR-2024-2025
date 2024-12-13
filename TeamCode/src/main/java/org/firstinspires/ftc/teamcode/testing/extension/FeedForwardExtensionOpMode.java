package org.firstinspires.ftc.teamcode.testing.extension;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.teamcode.commands.extension.MoveExtensionCommand;
import org.firstinspires.ftc.teamcode.subsystems.ExtensionSubsystem;

@TeleOp
public class FeedForwardExtensionOpMode extends OpMode {
    private ExtensionSubsystem elevator;
    private MoveExtensionCommand moveExtensionCommand;

    @Override
    public void init() {
        DcMotorEx motorElevator = hardwareMap.get(DcMotorEx.class, "motorExtension");
        elevator = new ExtensionSubsystem(motorElevator);

        moveExtensionCommand = new MoveExtensionCommand(elevator, 800);

        CommandScheduler.getInstance().reset();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        if (gamepad1.right_bumper) {
            CommandScheduler.getInstance().schedule(
                    new MoveExtensionCommand(elevator, 800)
            );
        } else if (gamepad1.dpad_up) {
            elevator.setTarget(elevator.getTarget() + 7);
        } else if (gamepad1.dpad_down) {
            elevator.setTarget(elevator.getTarget() - 7);
        } else if (gamepad1.x) {
            elevator.resetEncoder();
        } else if (gamepad1.b) {
            CommandScheduler.getInstance().schedule(new MoveExtensionCommand(elevator, 600));
        }

        elevator.moveElevator();

        telemetry.addData("Target Position", elevator.getTarget());
        telemetry.addData("Slide Position", elevator.getCurrentPosition());
        telemetry.update();

        CommandScheduler.getInstance().run();
    }
}