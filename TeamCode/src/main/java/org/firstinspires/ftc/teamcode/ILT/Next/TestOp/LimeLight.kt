package org.firstinspires.ftc.teamcode.robot.opmodes.test

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Data.Config.RobotConfig
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Shooter.Turret

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
            BulkReadComponent,
            BindingsComponent
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


        // ==================== TELEMETRY ====================
        telemetry.addLine("=== LIMELIGHT TEST ===")
        telemetry.addLine()

        telemetry.addData("LL Ready", RobotState.limelightReady)
        telemetry.addData("Has Target", RobotState.limelightHasTarget)
        telemetry.addLine()

        if (RobotState.limelightHasTarget) {
            telemetry.addLine("--- TARGET DATA ---")
            telemetry.addData("TX (horizontal)",  RobotState.limelightTx)
            telemetry.addData("TY (vertical)",  RobotState.limelightTy)
            telemetry.addData("TA (area)",  RobotState.limelightTa)
            telemetry.addLine()
            telemetry.addLine("--- CALCULATED DISTANCE ---")
            val distanceStr = RobotState.distanceToGoalLimelight
            telemetry.addData("Distance", distanceStr)
            telemetry.addLine()

            // Show what TX means
            when {
                RobotState.limelightTx > 2 -> telemetry.addData("Target is", ">> RIGHT")
                RobotState.limelightTx < -2 -> telemetry.addData("Target is", "<< LEFT")
                else -> telemetry.addData("Target is", "CENTERED")
            }
        } else {
            telemetry.addLine("NO TARGET VISIBLE")
            telemetry.addLine("Point camera at the goal!")
        }

        telemetry.addLine()
        telemetry.addLine("--- CALIBRATION VALUES ---")
        telemetry.addData(
            "Mount Angle (Dpad U/D)",
             RobotConfig.LimelightConfig.mountAngleDeg
        )
        telemetry.addData(
            "Lens Height (Dpad L/R)",
            RobotConfig.LimelightConfig.lensHeightIn
        )
        telemetry.addData("Goal Height", RobotConfig.LimelightConfig.goalHeightIn)
        telemetry.addLine()
        telemetry.addData("Motif Detected", RobotState.detectedMotif)
        telemetry.update()
    }
}