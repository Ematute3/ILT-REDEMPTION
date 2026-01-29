package org.firstinspires.ftc.teamcode.robot.opmodes.test

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.extensions.pedro.PedroDriverControlled
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Drive.DriveTrain
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.robot.data.config.RobotState


/**
 * Simple Drive Test
 *
 * Left Stick = Drive/Strafe
 * Right Stick X = Turn
 * A = Reset Pose to 0,0,0
 */
@TeleOp(name = "Test - Drive Only", group = "Test")
class DriveOnlyTestOpMode : NextFTCOpMode() {

    init {
        addComponents(
            PedroComponent(Constants::createFollower),
            SubsystemComponent(DriveTrain),
            BulkReadComponent,
            BindingsComponent
        )
    }

    override fun onInit() {
        telemetry.addLine("=== DRIVE TEST ===")
        telemetry.addLine("Left Stick = Drive")
        telemetry.addLine("Right Stick X = Turn")
        telemetry.update()
    }

    override fun onStartButtonPressed() {
        // DriveTrain.defaultCommand handles driving
        val driverControlled = PedroDriverControlled(
            -Gamepads.gamepad1.leftStickY,
            -Gamepads.gamepad1.leftStickX,
            -Gamepads.gamepad1.rightStickX,
            false  // false = field centric, true = robot centric
        )
        driverControlled()
    }

    override fun onUpdate() {
        // Reset pose


        telemetry.addLine("=== DRIVE TEST ===")
        telemetry.addData("Pose Valid", RobotState.poseValid)
        telemetry.addData("X", "%.1f".format(RobotState.currentX))
        telemetry.addData("Y", "%.1f".format(RobotState.currentY))
        telemetry.addData("Heading", "%.1f°".format(Math.toDegrees(RobotState.currentHeading)))
        telemetry.addLine()
        telemetry.addLine("A = Reset Pose")
        telemetry.addLine()
        telemetry.addData("Stick Y", "%.2f".format(gamepad1.left_stick_y))
        telemetry.addData("Stick X", "%.2f".format(gamepad1.left_stick_x))
        telemetry.addData("Turn", "%.2f".format(gamepad1.right_stick_x))
    }
}