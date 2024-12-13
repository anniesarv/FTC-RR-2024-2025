package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.teamcode.commands.drivetrain.FieldCentricCommand;
import org.firstinspires.ftc.teamcode.commands.elevator.MoveElevatorCommand;
import org.firstinspires.ftc.teamcode.commands.intakewrist.IntakeDown;
import org.firstinspires.ftc.teamcode.commands.intakewrist.IntakeUp;
import org.firstinspires.ftc.teamcode.commands.intakewrist.RunIntake;
import org.firstinspires.ftc.teamcode.commands.intakewrist.RunOuttake;
import org.firstinspires.ftc.teamcode.commands.intakewrist.StopIntake;
import org.firstinspires.ftc.teamcode.commands.outtakewrist.CloseClaw;
import org.firstinspires.ftc.teamcode.commands.outtakewrist.OpenClaw;
import org.firstinspires.ftc.teamcode.commands.outtakewrist.OuttakeDown;
import org.firstinspires.ftc.teamcode.commands.outtakewrist.OuttakeUp;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeWrist;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeWrist;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

@TeleOp
public class TeleOpGame extends LinearOpMode {

    private DriveSubsystem driveSubsystem;
    private FieldCentricCommand driveCommand;
    private ElevatorSubsystem elevator;

    private ExtensionSubsystem extensionSubsystem;
    private IntakeWrist intakeWrist;
    private OuttakeWrist outtakeWrist;
    private GamepadEx driver2;
    private RobotHardware robotHardware;

    @Override
    public void runOpMode() throws InterruptedException {
        driveSubsystem = new DriveSubsystem(hardwareMap);
        driveCommand = new FieldCentricCommand(driveSubsystem, gamepad1);

        DcMotorEx motorElevator = hardwareMap.get(DcMotorEx.class, "motorElevator");
        elevator = new ElevatorSubsystem(motorElevator);
        //moveElevatorCommand = new MoveElevatorCommand(elevator, 800);

        DcMotorEx motorExtension = hardwareMap.get(DcMotorEx.class, "motorExtension");
        extensionSubsystem = new ExtensionSubsystem(motorExtension);
        robotHardware = new RobotHardware();
        robotHardware.init(hardwareMap);
        intakeWrist = new IntakeWrist(robotHardware);
        outtakeWrist = new OuttakeWrist(robotHardware);

        driver2 = new GamepadEx(gamepad2);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        while (opModeIsActive()) {
            driveCommand.execute();

            if (gamepad1.right_bumper) {
                CommandScheduler.getInstance().schedule(new MoveElevatorCommand(elevator, 800));
            } else if (gamepad1.dpad_up) {
                elevator.setTarget(elevator.getTarget() + 15);
            } else if (gamepad1.dpad_down) {
                elevator.setTarget(elevator.getTarget() - 15);
            } else if (gamepad1.x) {
                elevator.resetEncoder();
            } else if (gamepad1.b) {
                CommandScheduler.getInstance().schedule(new MoveElevatorCommand(elevator, 600));
            }

            elevator.moveElevator();

              if (gamepad2.dpad_left) {
                extensionSubsystem.setTarget(extensionSubsystem.getTarget() + 15);
            } else if (gamepad2.dpad_right) {
                extensionSubsystem.setTarget(extensionSubsystem.getTarget() - 15);
            } else if (gamepad2.back) {
                extensionSubsystem.resetEncoder();
            }

            extensionSubsystem.moveExtension();

            // Intake and Outtake controls via second gamepad
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

            telemetry.addData("Heading (rad)", driveSubsystem.getHeading());
            telemetry.addData("Joystick (x, y, rx)", "%.2f, %.2f, %.2f", gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
            telemetry.addData("Elevator Target Position", elevator.getTarget());
            telemetry.addData("Extension Target", extensionSubsystem.getTarget());

            telemetry.addData("Slide Position", elevator.getCurrentPosition());
            telemetry.update();

            CommandScheduler.getInstance().run();
        }
    }
}
