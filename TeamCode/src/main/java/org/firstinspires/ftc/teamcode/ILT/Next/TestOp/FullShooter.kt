package org.firstinspires.ftc.teamcode.robot.opmodes.test

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import org.firstinspires.ftc.teamcode.ILT.Next.Commands.ShootCommands
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Data.Aimbot.AimbotTable
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Data.Config.RobotConfig
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Intake.Intake

import org.firstinspires.ftc.teamcode.robot.data.config.RobotState
import org.firstinspires.ftc.teamcode.robot.subsystems.intake.Gate

import org.firstinspires.ftc.teamcode.robot.subsystems.shooter.FlyWheel
import org.firstinspires.ftc.teamcode.robot.subsystems.shooter.Hood
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Shooter.Turret
import org.firstinspires.ftc.teamcode.robot.subsystems.vision.Limelight
import kotlin.math.abs

/**
 * ============================================================
 * FULL SHOOTER TEST OPMODE
 * ============================================================
 *
 * Tests ALL shooter components together:
 * - Turret aiming (Limelight)
 * - Hood position
 * - Flywheel velocity
 * - Intake feeding
 *
 * USE THIS TO:
 * 1. Test complete shooting sequence
 * 2. Tune AimbotTable values for each distance
 * 3. Practice before competition
 *
 * CONTROLS (Gamepad 1):
 *
 *   A              - Enable auto-aim (turret tracks target)
 *   B              - Disable auto-aim
 *
 *   Right Trigger  - Spin up flywheel
 *   Left Trigger   - Stop flywheel
 *
 *   Right Bumper   - SHOOT (feed ball when ready)
 *   Left Bumper    - Full auto shoot sequence
 *
 *   Dpad Up/Down   - Adjust manual distance (for tuning)
 *
 *   Left Stick X   - Manual turret
 *   Left Stick Y   - Manual hood
 *
 *   Y              - Zero turret encoder
 *
 * TUNING AIMBOT TABLE:
 *
 * 1. Set robot at known distance (measure with tape)
 * 2. Use Dpad to set that distance
 * 3. Enable auto-aim (A)
 * 4. Spin flywheel (Right Trigger)
 * 5. Adjust hood with Left Stick Y until shots go in
 * 6. Note the hood position
 * 7. If velocity wrong, check telemetry and adjust table
 * 8. Record values in AimbotTable.kt
 */
@TeleOp(name = "Test - Full Shooter", group = "Test")
class FullShooterTestOpMode : NextFTCOpMode() {

    init {
        addComponents(
            SubsystemComponent(
                Turret,
                FlyWheel,
                Hood,
                Limelight,
                Intake,
                Gate
            ),
            BulkReadComponent
        )
    }

    private var autoAimEnabled = false
    private var manualDistance = 48  // inches

    override fun onInit() {
        telemetry.addLine("=== FULL SHOOTER TEST ===")
        telemetry.addLine("Align turret FORWARD before INIT!")
        telemetry.update()
    }

    override fun onStartButtonPressed() {
        autoAimEnabled = false
        RobotState.flywheelOn = false
    }

    override fun onUpdate() {
        // ==================== AUTO-AIM CONTROL ====================
        if (gamepad1.a) autoAimEnabled = true
        if (gamepad1.b) autoAimEnabled = false

        // ==================== TURRET CONTROL ====================
        val turretManual = -gamepad1.left_stick_x.toDouble()

        if (abs(turretManual) > 0.1) {
            // Manual override
            Turret.setManualPowerTurret(turretManual * RobotConfig.TurretConfig.manualPowerFast)
        } else if (autoAimEnabled) {
            Turret.aimWithLimelight()
        } else {
            Turret.stop()
        }

        // ==================== HOOD CONTROL ====================
        val hoodManual = -gamepad1.left_stick_y.toDouble()

        if (abs(hoodManual) > 0.1) {
            // Manual hood adjust
            RobotState.hoodPosition = (RobotState.hoodPosition + hoodManual * 0.01)
                .coerceIn(0.0, 1.0)
        } else {
            // Use table values
            val values = AimbotTable.getValues(manualDistance.toDouble())
            if (values != null) {
                RobotState.hoodPosition = values.first
            }
        }

        // ==================== FLYWHEEL CONTROL ====================
        if (gamepad1.right_trigger > 0.5) {
            val values = AimbotTable.getValues(manualDistance.toDouble())
            if (values != null) {
                FlyWheel.setTargetVelocity(values.second)
            }
            RobotState.flywheelOn = true
        }
        if (gamepad1.left_trigger > 0.5) {
            RobotState.flywheelOn = false
        }

        // ==================== SHOOTING ====================
        // Manual feed
        if (gamepad1.right_bumper && RobotState.flywheelAtSpeed) {
            Intake.feed.run()
        } else if (!gamepad1.left_bumper) {
            Intake.stop.run()
        }

        // Full auto sequence
        if (gamepad1.left_bumper) {
            ShootCommands.simpleShoot.schedule()
        }

        // ==================== DISTANCE ADJUSTMENT ====================
        if (gamepad1.dpad_up) {
            manualDistance = (manualDistance + 12).coerceAtMost(144)
        }
        if (gamepad1.dpad_down) {
            manualDistance = (manualDistance - 12).coerceAtLeast(12)
        }

        // ==================== ZERO TURRET ====================
        if (gamepad1.y) {
            Turret.resetEncoderToZero()
        }

        // ==================== TELEMETRY ====================
        telemetry.addLine("=== FULL SHOOTER TEST ===")
        telemetry.addLine()

        // Status
        telemetry.addData("Auto-Aim", if (autoAimEnabled) "ON" else "OFF")
        telemetry.addData("Manual Distance", "$manualDistance inches")
        telemetry.addLine()

        // Ready status
        val turretReady = RobotState.turretAligned || !autoAimEnabled
        val flywheelReady = RobotState.flywheelAtSpeed
        telemetry.addData("Turret", if (turretReady) "✓ Ready" else "✗ Aiming...")
        telemetry.addData("Flywheel", if (flywheelReady) "✓ Ready" else "✗ Spinning...")
        telemetry.addData("READY TO SHOOT", if (turretReady && flywheelReady) "✓ YES!" else "✗ No")
        telemetry.addLine()

        // Current values
        telemetry.addLine("--- CURRENT VALUES ---")
        telemetry.addData("Turret Angle", "%.1f°".format(Turret.getYawDegrees()))
        telemetry.addData("Hood Position", "%.3f".format(RobotState.hoodPosition))
        telemetry.addData("Flywheel Velocity", "%.0f".format(RobotState.flywheelVelocity))
        telemetry.addData("Flywheel RPM", "%.0f".format(FlyWheel.getRPM()))
        telemetry.addLine()

        // Table values for current distance
        val tableValues = AimbotTable.getValues(manualDistance.toDouble())
        if (tableValues != null) {
            telemetry.addLine("--- TABLE VALUES @ ${manualDistance}\" ---")
            telemetry.addData("Table Hood", "%.3f".format(tableValues.first))
            telemetry.addData("Table Velocity", "%.0f".format(tableValues.second))
        }

        telemetry.addLine()
        telemetry.addLine("--- LIMELIGHT ---")
        telemetry.addData("Has Target", RobotState.limelightHasTarget)
        if (RobotState.limelightHasTarget) {
            telemetry.addData("TX", "%.2f°".format(RobotState.limelightTx))
            telemetry.addData("LL Distance",
                RobotState.distanceToGoalLimelight?.let { "%.1f\"".format(it) } ?: "N/A")
        }
    }
}