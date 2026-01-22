package org.firstinspires.ftc.teamcode.robot.opmodes

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import dev.nextftc.bindings.button
import org.firstinspires.ftc.teamcode.ILT.Next.Commands.ShootCommands
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Data.Config.RobotConfig
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Data.Enums.Alliance
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Intake.Intake

import org.firstinspires.ftc.teamcode.robot.data.config.RobotState

import org.firstinspires.ftc.teamcode.robot.subsystems.drive.DriveTrain
import org.firstinspires.ftc.teamcode.robot.subsystems.intake.Gate
import org.firstinspires.ftc.teamcode.robot.subsystems.shooter.FlyWheel
import org.firstinspires.ftc.teamcode.robot.subsystems.shooter.Hood
import org.firstinspires.ftc.teamcode.robot.subsystems.shooter.OuttakeController
import org.firstinspires.ftc.teamcode.robot.subsystems.shooter.Turret
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.robot.subsystems.vision.Limelight
import kotlin.math.abs

/**
 * Main TeleOp using Pedro driving.
 *
 * Gamepad 1 (Driver):
 *   Left stick - Drive/Strafe
 *   Right stick X - Turn
 *   Right bumper - Intake
 *   Left trigger - Reverse intake
 *   Back - IMU reset
 *
 * Gamepad 2 (Operator):
 *   Dpad up/down - Manual aim distance
 *   Left/Right bumper - Turret manual
 *   Right stick X - Turret fine control
 *   Left stick Y - Hood adjust
 *   A - Auto shoot sequence
 *   B - Emergency stop
 *   X - Limelight mode
 *   Y - Odometry mode
 *   Back - Idle mode
 */
@TeleOp(name = "Main TeleOp", group = "Competition")
class MainTeleOp : NextFTCOpMode() {

    init {
        addComponents(
            PedroComponent(Constants::createFollower),
            SubsystemComponent(
                DriveTrain,
                FlyWheel,
                Intake,
                Turret,
                Hood,
                Limelight,
                OuttakeController,
                Gate
            ),
            BulkReadComponent,
            BindingsComponent
        )
    }

    override fun onInit() {
        // Set alliance (could be from dashboard or auto-detect)
        RobotConfig.alliance = Alliance.RED

        // Reset state
        RobotState.reset()
    }

    override fun onStartButtonPressed() {
        // Bind gamepad controls
        bindDriverControls()
        bindOperatorControls()
    }

    private fun bindDriverControls() {
        // Intake controls
        button { gamepad1.right_bumper }
            .whenTrue(Intake.run)
            .whenBecomesFalse(Intake.stop)

        // Reverse intake on left trigger
        button { gamepad1.left_trigger > 0.1f }
            .whenTrue(Intake.reverse)
            .whenBecomesFalse(Intake.stop)

        // IMU reset
        button { gamepad1.back }
            .whenBecomesTrue { DriveTrain.resetImu() }
    }

    private fun bindOperatorControls() {
        // Manual aim adjustment
        button { gamepad2.dpad_up }
            .whenBecomesTrue(OuttakeController.aimUp)

        button { gamepad2.dpad_down }
            .whenBecomesTrue(OuttakeController.aimDown)

        // Turret manual control
        button { gamepad2.left_bumper }
            .whenTrue(Turret.spinLeft)
            .whenBecomesFalse(Turret.stopTurret)

        button { gamepad2.right_bumper }
            .whenTrue(Turret.spinRight)
            .whenBecomesFalse(Turret.stopTurret)

        // Mode selection
        button { gamepad2.x }
            .whenBecomesTrue(OuttakeController.autoLimelightMode)

        button { gamepad2.y }
            .whenBecomesTrue(OuttakeController.autoOdometryMode)

        button { gamepad2.back }
            .whenBecomesTrue(OuttakeController.idleMode)

        // Shooting - A button
        button { gamepad2.a }
            .whenBecomesTrue { ShootCommands.fullAutoShoot.schedule() }

        // Emergency stop - B button
        button { gamepad2.b }
            .whenBecomesTrue { ShootCommands.emergencyStop.schedule() }

        // Flywheel spin toggle - right trigger
        button { gamepad2.right_trigger > 0.5f }
            .whenBecomesTrue {
                if (RobotState.flywheelOn) {
                    FlyWheel.stop.schedule()
                } else {
                    FlyWheel.spin.schedule()
                }
            }
    }

    override fun onUpdate() {
        // Update turret manual power from gamepad
        val turretInput = gamepad2.right_stick_x.toDouble()
        if (abs(turretInput) > 0.1) {
            Turret.manualPower = turretInput * RobotConfig.TurretConfig.manualPowerFast
        } else {
            Turret.manualPower = 0.0
        }

        // Hood manual adjust from left stick Y
        val hoodInput = gamepad2.left_stick_y.toDouble()
        if (abs(hoodInput) > 0.1) {
            RobotState.hoodPosition = (RobotState.hoodPosition + hoodInput * 0.01)
                .coerceIn(0.0, 1.0)
        }

        // Display status
        telemetry.addData("=== ROBOT STATUS ===", "")
        telemetry.addData("Alliance", RobotConfig.alliance)
        telemetry.addData("Can Shoot", RobotState.canShoot())
        telemetry.addData("Flywheel On", RobotState.flywheelOn)
        telemetry.addData("Turret Aligned", RobotState.turretAligned)
    }
}