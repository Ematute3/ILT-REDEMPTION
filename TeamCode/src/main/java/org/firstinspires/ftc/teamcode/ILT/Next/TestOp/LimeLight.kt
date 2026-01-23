package org.firstinspires.ftc.teamcode.ILT.Next.TestOp

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Data.Config.RobotConfig

import org.firstinspires.ftc.teamcode.robot.data.config.RobotState
import org.firstinspires.ftc.teamcode.robot.subsystems.vision.Limelight


/**
 * ============================================================
 * LIMELIGHT TEST OPMODE
 * ============================================================
 *
 * Use this to verify Limelight is working and calibrate distance.
 *
 * SETUP:
 * 1. Point robot at the target
 * 2. Measure actual distance with tape measure
 * 3. Compare to calculated distance
 * 4. Adjust mounting angle/height in RobotConfig if needed
 *
 * WHAT TO CHECK:
 *
 * 1. "Has Target" should be TRUE when pointing at target
 * 2. TX should be ~0 when target is centered
 * 3. TX positive = target is to the RIGHT
 * 4. TX negative = target is to the LEFT
 * 5. Distance should match your tape measure
 *
 * CALIBRATING DISTANCE:
 *
 * If distance is wrong, adjust these in RobotConfig.Limelight:
 * - mountAngleDeg: Angle camera is tilted up
 * - lensHeightIn: Height of camera lens from ground
 * - goalHeightIn: Height of target from ground
 *
 * Formula: distance = (goalHeight - lensHeight) / tan(mountAngle + TY)
 *
 * CONTROLS:
 *
 *   Dpad Up/Down   - Adjust mount angle
 *   Dpad Left/Right - Adjust lens height
 */
@TeleOp(name = "Limelight", group = "Test")
class LimelightTest : NextFTCOpMode() {

    init {
        addComponents(
            SubsystemComponent(Limelight),
            BulkReadComponent
        )
    }

    override fun onInit() {
        telemetry.addLine("=== LIMELIGHT TEST ===")
        telemetry.addLine("Point at target to test")
        telemetry.update()
    }

    override fun onStartButtonPressed() {
        // Nothing needed
    }

    override fun onUpdate() {
        // ==================== CALIBRATION ADJUSTMENTS ====================
        if (gamepad1.dpad_up) {
            RobotConfig.LimelightConfig.mountAngleDeg += 0.5
        }
        if (gamepad1.dpad_down) {
            RobotConfig.LimelightConfig.mountAngleDeg -= 0.5
        }
        if (gamepad1.dpad_right) {
            RobotConfig.LimelightConfig.lensHeightIn += 0.25
        }
        if (gamepad1.dpad_left) {
            RobotConfig.LimelightConfig.lensHeightIn -= 0.25
        }

        // ==================== TELEMETRY ====================
        telemetry.addLine("=== LIMELIGHT TEST ===")
        telemetry.addLine()

        telemetry.addData("LL Ready", RobotState.limelightReady)
        telemetry.addData("Has Target", RobotState.limelightHasTarget)
        telemetry.addLine()

        if (RobotState.limelightHasTarget) {
            telemetry.addLine("--- TARGET DATA ---")
            telemetry.addData("TX (horizontal)", "%.2f°".format(RobotState.limelightTx))
            telemetry.addData("TY (vertical)", "%.2f°".format(RobotState.limelightTy))
            telemetry.addData("TA (area)", "%.2f%%".format(RobotState.limelightTa))
            telemetry.addLine()
            telemetry.addLine("--- CALCULATED DISTANCE ---")
            telemetry.addData("Distance",
                RobotState.distanceToGoalLimelight?.let { "%.1f inches".format(it) } ?: "N/A")
            telemetry.addLine()

            // Show what TX means
            when {
                RobotState.limelightTx > 2 -> telemetry.addData("Target is", "→ RIGHT")
                RobotState.limelightTx < -2 -> telemetry.addData("Target is", "← LEFT")
                else -> telemetry.addData("Target is", "✓ CENTERED")
            }
        } else {
            telemetry.addLine("NO TARGET VISIBLE")
            telemetry.addLine("Point camera at the goal!")
        }

        telemetry.addLine()
        telemetry.addLine("--- CALIBRATION VALUES ---")
        telemetry.addData("Mount Angle (Dpad U/D)", "%.1f°".format(RobotConfig.LimelightConfig.mountAngleDeg))
        telemetry.addData("Lens Height (Dpad L/R)", "%.2f in".format(RobotConfig.LimelightConfig.lensHeightIn))
        telemetry.addData("Goal Height", "%.1f in".format(RobotConfig.LimelightConfig.goalHeightIn))
        telemetry.addLine()
        telemetry.addData("Motif Detected", RobotState.detectedMotif)
    }
}