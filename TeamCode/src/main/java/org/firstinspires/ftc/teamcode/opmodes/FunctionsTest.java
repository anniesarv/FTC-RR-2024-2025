package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
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
import org.firstinspires.ftc.teamcode.subsystems.IntakeWrist;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeWrist;

@TeleOp
public class FunctionsTest extends OpMode {

    private RobotHardware robotHardware;
    private IntakeWrist intakeWrist;
    private OuttakeWrist outtakeWrist;

    private GamepadEx driver;

    @Override
    public void init() {
        CommandScheduler.getInstance().reset();
        robotHardware = new RobotHardware();
        robotHardware.init(hardwareMap);

        intakeWrist = new IntakeWrist(robotHardware);
        outtakeWrist = new OuttakeWrist(robotHardware);

        driver = new GamepadEx(gamepad1);

        /*
        CommandScheduler.getInstance().schedule(
                new TeleOpFunctions(intakeWrist, outtakeWrist, gamepad1)
        );

         */
    }

    @Override
    public void loop() {
        //run intake with button A
        new GamepadButton(driver, GamepadKeys.Button.A).whenPressed(
                new RunIntake(intakeWrist)
        ).whenReleased(
                new StopIntake(intakeWrist)
        );

        //run outtake with button A
        new GamepadButton(driver, GamepadKeys.Button.B).whenPressed(
                new RunOuttake(intakeWrist)
        ).whenReleased(
                new StopIntake(intakeWrist)
        );

        //Intake down when d pad down
        new GamepadButton(driver, GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new IntakeDown(intakeWrist)
        );

        //Intake up when d pad up
        new GamepadButton(driver, GamepadKeys.Button.DPAD_UP).whenPressed(
                       new IntakeUp(intakeWrist)
        );

        //Outtake up when left bumper
        new GamepadButton(driver, GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                new OuttakeUp(outtakeWrist)
        );

        //Outtake down when right bumper
        new GamepadButton(driver, GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                new OuttakeDown(outtakeWrist)
        );

        new GamepadButton(driver, GamepadKeys.Button.X).whenPressed(
                new OpenClaw(outtakeWrist)
        );

        new GamepadButton(driver, GamepadKeys.Button.Y).whenPressed(
                new CloseClaw(outtakeWrist)
        );
        CommandScheduler.getInstance().run();


    }
}
