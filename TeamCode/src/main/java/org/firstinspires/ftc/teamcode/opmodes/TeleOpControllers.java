package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeWrist;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeWrist;
import org.firstinspires.ftc.teamcode.commands.drivetrain.FieldCentricCommand;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOpControllers extends OpMode {

    // Drivebase subsystems and commands
    private DriveSubsystem driveSubsystem;
    private FieldCentricCommand driveCommand;

    // Function subsystems
    private RobotHardware robotHardware;
    private IntakeWrist intakeWrist;
    private OuttakeWrist outtakeWrist;

    // Gamepad objects
    private GamepadEx driver1; // for drivebase control
    private GamepadEx driver2; // for functions control

    @Override
    public void init() {
        // Initialize drive subsystem and command
        driveSubsystem = new DriveSubsystem(hardwareMap);
        driveCommand = new FieldCentricCommand(driveSubsystem, gamepad1);

        // Initialize function subsystems
        robotHardware = new RobotHardware();
        robotHardware.init(hardwareMap);

        intakeWrist = new IntakeWrist(robotHardware);
        outtakeWrist = new OuttakeWrist(robotHardware);

        // Gamepad objects for function control
        driver2 = new GamepadEx(gamepad2);
        driver1 = new GamepadEx(gamepad1);

        CommandScheduler.getInstance().reset();
    }

    @Override
    public void loop() {
        // Drivebase control via gamepad1 (field-centric)
        driveCommand.execute();

        // Function controls via gamepad2
        // Run intake with button A
        new GamepadButton(driver2, GamepadKeys.Button.A).whenPressed(
                new RunIntake(intakeWrist)
        ).whenReleased(
                new StopIntake(intakeWrist)
        );

        // Run outtake with button B
        new GamepadButton(driver2, GamepadKeys.Button.B).whenPressed(
                new RunOuttake(intakeWrist)
        ).whenReleased(
                new StopIntake(intakeWrist)
        );

        // Intake down when DPAD down
        new GamepadButton(driver2, GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new IntakeDown(intakeWrist)
        );

        // Intake up when DPAD up
        new GamepadButton(driver2, GamepadKeys.Button.DPAD_UP).whenPressed(
                new IntakeUp(intakeWrist)
        );

        // Outtake up when left bumper
        new GamepadButton(driver2, GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                new OuttakeUp(outtakeWrist)
        );

        // Outtake down when right bumper
        new GamepadButton(driver2, GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                new OuttakeDown(outtakeWrist)
        );

        // Open claw with X button
        new GamepadButton(driver2, GamepadKeys.Button.X).whenPressed(
                new OpenClaw(outtakeWrist)
        );

        // Close claw with Y button
        new GamepadButton(driver2, GamepadKeys.Button.Y).whenPressed(
                new CloseClaw(outtakeWrist)
        );

        CommandScheduler.getInstance().run();

        // Telemetry for debugging
        telemetry.addData("Heading (rad)", driveSubsystem.getHeading());
        telemetry.addData("Joystick (x, y, rx)", "%.2f, %.2f, %.2f", gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
        telemetry.update();
    }
}
