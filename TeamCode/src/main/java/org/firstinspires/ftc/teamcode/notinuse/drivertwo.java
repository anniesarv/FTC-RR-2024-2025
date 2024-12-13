package org.firstinspires.ftc.teamcode.testing.OPS;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.teamcode.commands.extension.MoveExtensionCommand;
import org.firstinspires.ftc.teamcode.commands.intakewrist.IntakeDown;
import org.firstinspires.ftc.teamcode.commands.intakewrist.IntakeUp;
import org.firstinspires.ftc.teamcode.commands.intakewrist.RunIntake;
import org.firstinspires.ftc.teamcode.commands.intakewrist.RunOuttake;
import org.firstinspires.ftc.teamcode.commands.intakewrist.StopIntake;
import org.firstinspires.ftc.teamcode.commands.outtakewrist.CloseClaw;
import org.firstinspires.ftc.teamcode.commands.outtakewrist.OpenClaw;
import org.firstinspires.ftc.teamcode.commands.outtakewrist.OuttakeDown;
import org.firstinspires.ftc.teamcode.commands.outtakewrist.OuttakeUp;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeWrist;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeWrist;

@TeleOp
public class drivertwo extends OpMode {

    private ExtensionSubsystem elevator;
    private IntakeWrist intakeWrist;
    private OuttakeWrist outtakeWrist;
    private GamepadEx driver2;
    private RobotHardware robotHardware;

    @Override
    public void init() {
        DcMotorEx motorElevator = hardwareMap.get(DcMotorEx.class, "motorExtension");
        elevator = new ExtensionSubsystem(motorElevator);

        robotHardware = new RobotHardware();
        robotHardware.init(hardwareMap);
        intakeWrist = new IntakeWrist(robotHardware);
        outtakeWrist = new OuttakeWrist(robotHardware);

        driver2 = new GamepadEx(gamepad2);

        CommandScheduler.getInstance().reset();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        // Extension control
        if (gamepad2.options) {
            CommandScheduler.getInstance().schedule(new MoveExtensionCommand(elevator, 0));
        } else if (gamepad2.dpad_left) {
            elevator.setTarget(elevator.getTarget() + 7);
        } else if (gamepad2.dpad_right) {
            elevator.setTarget(elevator.getTarget() - 7);
        } else if (gamepad2.back) {
            elevator.resetEncoder();
       // } else if (gamepad1.b) {
           // CommandScheduler.getInstance().schedule(new MoveExtensionCommand(elevator, 600));
        }
        elevator.moveExtension();

        new GamepadButton(driver2, GamepadKeys.Button.A).whenPressed(
                new RunIntake(intakeWrist)
        ).whenReleased(
                new StopIntake(intakeWrist)
        );

        new GamepadButton(driver2, GamepadKeys.Button.B).whenPressed(
                new RunOuttake(intakeWrist)
        ).whenReleased(
                new StopIntake(intakeWrist)
        );

        new GamepadButton(driver2, GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new IntakeDown(intakeWrist)
        );

        new GamepadButton(driver2, GamepadKeys.Button.DPAD_UP).whenPressed(
                new IntakeUp(intakeWrist)
        );

        new GamepadButton(driver2, GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                new OuttakeUp(outtakeWrist)
        );

        new GamepadButton(driver2, GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                new OuttakeDown(outtakeWrist)
        );

        new GamepadButton(driver2, GamepadKeys.Button.X).whenPressed(
                new OpenClaw(outtakeWrist)
        );

        new GamepadButton(driver2, GamepadKeys.Button.Y).whenPressed(
                new CloseClaw(outtakeWrist)
        );

        telemetry.addData("Target Position", elevator.getTarget());
        telemetry.addData("Slide Position", elevator.getCurrentPosition());
        telemetry.update();

        CommandScheduler.getInstance().run();
    }
}
