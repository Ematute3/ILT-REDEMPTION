package org.firstinspires.ftc.teamcode.ILT.Next.TestOp

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Data.Config.RobotConfig
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Shooter.Turret

import org.firstinspires.ftc.teamcode.robot.data.config.RobotState

import org.firstinspires.ftc.teamcode.robot.subsystems.vision.Limelight

import kotlin.math.abs

/**
 * ============================================================
 * TURRET + LIMELIGHT AIMING TEST
 * ============================================================
 *
 * Tests the turret tracking a target using Limelight.
 *
 * SETUP:
 * 1. Align turret forward, press INIT
 * 2. Point robot toward target
 * 3. Enable auto-aim and watch turret track!
 *
 * CONTROLS (Gamepad 1):
 *
 *   A              - Enable Limelight auto-aim
 *   B              - Disable auto-aim (stop)
 *   Left Stick X   - Manual control (overrides auto)
 *
 *   Dpad Up/Down   - Adjust kP (aiming speed)
 *   Dpad L/R       - Adjust alignment tolerance
 *
 *   Right Bumper   - Zero turret encoder
 *
 * TUNING PROCESS:
 *
 * 1. Enable auto-aim (A button)
 * 2. Watch turret track the target
 * 3. If too slow: increase kP (Dpad Up)
 * 4. If oscillates back and forth: decrease kP (Dpad Down)
 * 5. Ideal: Quickly centers and holds steady
 */
@TeleOp(name = "Test - Turret Aiming", group = "Test")
class TurretAimingTestOpMode : NextFTCOpMode() {

    init {
        addComponents(
            SubsystemComponent(Turret, Limelight),
            BulkReadComponent
        )
    }

    private var autoAimEnabled = false

    override fun onInit() {
        telemetry.addLine("=== TURRET AIMING TEST ===")
        telemetry.addLine("Align turret FORWARD before INIT!")
        telemetry.update()
    }

    override fun onStartButtonPressed() {
        autoAimEnabled = false
    }

    override fun onUpdate() {
        // ==================== MODE SELECTION ====================
        if (gamepad1.a) {
            autoAimEnabled = true
        }
        if (gamepad1.b) {
            autoAimEnabled = false
            Turret.stop()
        }

        // ==================== CONTROL ====================
        val manualInput = -gamepad1.left_stick_x.toDouble()

        if (abs(manualInput) > 0.1) {
            // Manual override
            autoAimEnabled = false
            Turret.setManualPowerTurret(manualInput * RobotConfig.TurretConfig.manualPowerFast)
        } else if (autoAimEnabled) {
            // Auto-aim using Limelight
            Turret.aimWithLimelight()
        } else {
            Turret.stop()
        }

        // ==================== ZERO ENCODER ====================
        if (gamepad1.right_bumper) {
            Turret.resetEncoderToZero()
        }

        // ==================== TUNING ====================
        if (gamepad1.dpad_up) {
            Turret.kP_limelight += 0.005
        }
        if (gamepad1.dpad_down) {
            Turret.kP_limelight = (Turret.kP_limelight - 0.005).coerceAtLeast(0.0)
        }
        if (gamepad1.dpad_right) {
            RobotConfig.LimelightConfig.alignmentToleranceDeg += 0.5
        }
        if (gamepad1.dpad_left) {
            RobotConfig.LimelightConfig.alignmentToleranceDeg =
                (RobotConfig.LimelightConfig.alignmentToleranceDeg - 0.5).coerceAtLeast(0.5)
        }

        // ==================== TELEMETRY ====================
        telemetry.addLine("=== TURRET AIMING TEST ===")
        telemetry.addLine()

        // Status
        telemetry.addData("Mode", if (autoAimEnabled) "AUTO AIM" else "MANUAL/OFF")
        telemetry.addData("Has Target", RobotState.limelightHasTarget)
        telemetry.addLine()

        // Aiming data
        if (RobotState.limelightHasTarget) {
            telemetry.addData("TX (error)", "%.2f°".format(RobotState.limelightTx))
            telemetry.addData("Turret Angle", "%.1f°".format(Turret.getYawDegrees()))
            telemetry.addData("ALIGNED", if (RobotState.turretAligned) "✓ YES!" else "✗ No")

            // Visual indicator
            val tx = RobotState.limelightTx
            val indicator = when {
                tx < -10 -> "◀◀◀◀◀ LEFT"
                tx < -5 -> "◀◀◀ LEFT"
                tx < -2 -> "◀ left"
                tx < 2 -> "▶◀ CENTERED ◀▶"
                tx < 5 -> "right ▶"
                tx < 10 -> "RIGHT ▶▶▶"
                else -> "RIGHT ▶▶▶▶▶"
            }
            telemetry.addData("Direction", indicator)
        } else {
            telemetry.addLine("NO TARGET - Point at goal!")
        }

        telemetry.addLine()
        telemetry.addLine("--- TUNING ---")
        telemetry.addData("kP (Dpad U/D)", "%.4f".format(Turret.kP_limelight))
        telemetry.addData("Tolerance (Dpad L/R)", "%.1f°".format(RobotConfig.LimelightConfig.alignmentToleranceDeg))
        telemetry.addLine()
        telemetry.addLine("A=AutoAim  B=Stop  Stick=Manual")
    }
}