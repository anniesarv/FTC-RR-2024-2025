package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.HorizontalArm;
import org.firstinspires.ftc.teamcode.commands.MoveHorizontalArmToPosition;

@TeleOp
public class HorizontalArmTest extends LinearOpMode {

    private RobotHardware robotHardware;
    private HorizontalArm horizontalArm;
    private int targetPosition = 0; // Current target position
    private final int POSITION_INCREMENT = 10; // Amount to increment/decrement

    @Override
    public void runOpMode() {
        robotHardware = new RobotHardware();
        robotHardware.init(hardwareMap); // Ensure to call init() method

        horizontalArm = new HorizontalArm(robotHardware);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.right_trigger > 0.1) {
                targetPosition += POSITION_INCREMENT; // Move forward
            }
            if (gamepad1.left_trigger > 0.1) {
                targetPosition -= POSITION_INCREMENT; // Move backward
            }

            targetPosition = Math.max(0, Math.min(targetPosition, 1000)); // Example: 0 to 1000 range

            CommandScheduler.getInstance().schedule(new MoveHorizontalArmToPosition(horizontalArm, targetPosition));

            CommandScheduler.getInstance().run();

            telemetry.addData("Target Position", targetPosition);
            telemetry.update();
        }
    }
}
