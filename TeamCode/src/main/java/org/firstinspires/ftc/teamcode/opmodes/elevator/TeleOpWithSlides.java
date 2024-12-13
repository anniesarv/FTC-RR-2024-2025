package org.firstinspires.ftc.teamcode.opmodes.elevator;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;

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

public class TeleOpWithSlides extends OpMode {

    // Drivebase subsystems and commands
    private DriveSubsystem driveSubsystem;
    private FieldCentricCommand driveCommand;

    // Function subsystems
    private RobotHardware robotHardware;
    private IntakeWrist intakeWrist;
    private OuttakeWrist outtakeWrist;

    // Extension motor and PID controller
    private DcMotor motorExtension;
    private PIDController extensionController;

    // PID constants and state variables
    private static final double EXTENSION_P = 0.005, EXTENSION_I = 0, EXTENSION_D = 0.00001;
    private static final double EXTENSION_KG = 0.1; // Gravity compensation
    private int extensionTarget = 0;

    // Gamepad objects
    private GamepadEx driver1;
    private GamepadEx driver2;

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

        // Initialize extension hardware and PID controller
        motorExtension = hardwareMap.get(DcMotorEx.class, "motorExtension");
        motorExtension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorExtension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        extensionController = new PIDController(EXTENSION_P, EXTENSION_I, EXTENSION_D);

        // Gamepad objects for function control
        driver2 = new GamepadEx(gamepad2);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        CommandScheduler.getInstance().reset();
    }

    @Override
    public void loop() {
        // Drivebase control
        driveCommand.execute();

        // Extension control
        extensionController.setPID(EXTENSION_P, EXTENSION_I, EXTENSION_D);
        int extensionPosition = motorExtension.getCurrentPosition();

        if (gamepad1.left_bumper) {
            extensionTarget = 800;
        }
        if (gamepad1.dpad_left) {
            extensionTarget += 7;
        } else if (gamepad1.dpad_right) {
            extensionTarget -= 7;
        }

        extensionTarget = Math.max(-100, Math.min(extensionTarget, 1000));

        if (gamepad1.x) {
            motorExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            extensionTarget = 0;
        }

        if (gamepad1.b) {
            extensionTarget = 600;
        }

        double extensionPid = extensionController.calculate(extensionPosition, extensionTarget);
        double extensionPower = extensionPid + EXTENSION_KG;
        motorExtension.setPower(extensionPower);

        // Telemetry
        telemetry.addData("Extension Target", extensionTarget);
        telemetry.addData("Extension Position", extensionPosition);
        telemetry.addData("Extension PID Output", extensionPid);
        telemetry.addData("Extension Power", extensionPower);

        CommandScheduler.getInstance().run();
        telemetry.update();
    }
}
