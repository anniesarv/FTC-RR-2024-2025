package org.firstinspires.ftc.teamcode.testing.OPS;

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
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeWrist;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeWrist;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

@TeleOp
public class TeleTwo extends LinearOpMode {

    // Drive and Elevator Subsystems
    private DriveSubsystem driveSubsystem;
    private FieldCentricCommand driveCommand;
    private ElevatorSubsystem elevator;
    private MoveElevatorCommand moveElevatorCommand;

    // Other subsystems
    private ExtensionSubsystem extension;
    private IntakeWrist intakeWrist;
    private OuttakeWrist outtakeWrist;

    // Gamepad controls
    private GamepadEx driver1;
    private GamepadEx driver2;
    private RobotHardware robotHardware;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the subsystems
        driveSubsystem = new DriveSubsystem(hardwareMap);
        driveCommand = new FieldCentricCommand(driveSubsystem, gamepad1);

        // Initialize Elevator and Extension Subsystems
        DcMotorEx motorElevator = hardwareMap.get(DcMotorEx.class, "motorElevator");
        elevator = new ElevatorSubsystem(motorElevator);
        moveElevatorCommand = new MoveElevatorCommand(elevator, 800);

        DcMotorEx motorExtension = hardwareMap.get(DcMotorEx.class, "motorExtension");
        extension = new ExtensionSubsystem(motorExtension);

        // Initialize RobotHardware (this will provide intake and outtake components)
        robotHardware = new RobotHardware();
        robotHardware.init(hardwareMap);

        // Initialize Intake and Outtake Subsystems
        intakeWrist = new IntakeWrist(robotHardware); // Pass RobotHardware to IntakeWrist
        outtakeWrist = new OuttakeWrist(robotHardware); // Pass RobotHardware to OuttakeWrist

        // Initialize gamepad input
        driver1 = new GamepadEx(gamepad1);
        driver2 = new GamepadEx(gamepad2);

        // Setup telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // Execute Drive Command
            driveCommand.execute();

            // Control the elevator (driver 1)
            if (gamepad1.right_bumper) {
                CommandScheduler.getInstance().schedule(new MoveElevatorCommand(elevator, 800));
            } else if (gamepad1.dpad_up) {
                elevator.setTarget(elevator.getTarget() + 20);
            } else if (gamepad1.dpad_down) {
                elevator.setTarget(elevator.getTarget() - 20);
            } else if (gamepad1.x) {
                elevator.resetEncoder();
            }

            elevator.moveElevator();

            // Control the extension (driver 2)
            if (gamepad2.dpad_left) {
                extension.setTarget(extension.getTarget() + 7);
            } else if (gamepad2.dpad_right) {
                extension.setTarget(extension.getTarget() - 7);
            } else if (gamepad2.back) {
                extension.resetEncoder();
            }
            extension.moveExtension();

            // Intake and Outtake control (driver 2)
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

            // Telemetry update
            telemetry.addData("Drive Heading", driveSubsystem.getHeading());
            telemetry.addData("Elevator Target", elevator.getTarget());
            telemetry.addData("Elevator Position", elevator.getCurrentPosition());
            telemetry.addData("Extension Target", extension.getTarget());
            telemetry.addData("Extension Position", extension.getCurrentPosition());
            telemetry.update();

            // Run the Command Scheduler
            CommandScheduler.getInstance().run();
        }
    }
}
