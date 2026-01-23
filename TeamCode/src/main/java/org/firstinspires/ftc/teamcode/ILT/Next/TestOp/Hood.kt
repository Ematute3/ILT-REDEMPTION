package org.firstinspires.ftc.teamcode.ILT.Next.TestOp

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import org.firstinspires.ftc.teamcode.robot.data.config.RobotState
import org.firstinspires.ftc.teamcode.robot.subsystems.shooter.Hood
import kotlin.math.abs

/**
 * ============================================================
 * HOOD TEST OPMODE
 * ============================================================
 *
 * Use this to test hood servo positions and find the range.
 *
 * CONTROLS (Gamepad 1):
 *
 *   Left Stick Y   - Adjust hood position smoothly
 *   A              - Go to 0.0 (min)
 *   B              - Go to 0.25
 *   X              - Go to 0.5 (middle)
 *   Y              - Go to 0.75
 *   Right Bumper   - Go to 1.0 (max)
 *
 *   Dpad Up/Down   - Fine adjust ±0.01
 *
 * FINDING YOUR RANGE:
 *
 * 1. Press A to go to 0.0 - note the hood angle
 * 2. Press Right Bumper for 1.0 - note the hood angle
 * 3. Find the useful range (where shots actually change)
 * 4. Record positions for each distance in AimbotTable
 */
@TeleOp(name = "Test - Hood", group = "Test")
class HoodTestOpMode : NextFTCOpMode() {

    init {
        addComponents(
            SubsystemComponent(Hood),
            BulkReadComponent
        )
    }

    override fun onInit() {
        telemetry.addLine("=== HOOD TEST ===")
        telemetry.addLine("Use stick or buttons to move hood")
        telemetry.update()
    }

    override fun onStartButtonPressed() {
        RobotState.hoodPosition = 0.5  // Start in middle
    }

    override fun onUpdate() {
        // ==================== SMOOTH CONTROL ====================
        val stickInput = -gamepad1.left_stick_y.toDouble()  // Up = increase
        if (abs(stickInput) > 0.1) {
            RobotState.hoodPosition = (RobotState.hoodPosition + stickInput * 0.01)
                .coerceIn(0.0, 1.0)
        }

        // ==================== PRESET POSITIONS ====================
        if (gamepad1.a) RobotState.hoodPosition = 0.0
        if (gamepad1.b) RobotState.hoodPosition = 0.25
        if (gamepad1.x) RobotState.hoodPosition = 0.5
        if (gamepad1.y) RobotState.hoodPosition = 0.75
        if (gamepad1.right_bumper) RobotState.hoodPosition = 1.0

        // ==================== FINE ADJUSTMENT ====================
        if (gamepad1.dpad_up) {
            RobotState.hoodPosition = (RobotState.hoodPosition + 0.01).coerceAtMost(1.0)
        }
        if (gamepad1.dpad_down) {
            RobotState.hoodPosition = (RobotState.hoodPosition - 0.01).coerceAtLeast(0.0)
        }

        // ==================== TELEMETRY ====================
        telemetry.addLine("=== HOOD TEST ===")
        telemetry.addLine()
        telemetry.addData("Hood Position", "%.3f".format(RobotState.hoodPosition))
        telemetry.addLine()

        // Visual bar
        val barLength = 20
        val filledLength = (RobotState.hoodPosition * barLength).toInt()
        val bar = "█".repeat(filledLength) + "░".repeat(barLength - filledLength)
        telemetry.addData("Visual", "[$bar]")

        telemetry.addLine()
        telemetry.addLine("--- CONTROLS ---")
        telemetry.addData("Left Stick Y", "Smooth adjust")
        telemetry.addData("A/B/X/Y/RB", "0.0/0.25/0.5/0.75/1.0")
        telemetry.addData("Dpad U/D", "Fine ±0.01")
    }
}