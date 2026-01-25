package org.firstinspires.ftc.teamcode.ILT.Next.TestOp

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.ftc.ActiveOpMode
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import org.firstinspires.ftc.teamcode.robot.subsystems.intake.Gate
import org.firstinspires.ftc.teamcode.robot.subsystems.shooter.Hood
import org.firstinspires.ftc.teamcode.robot.data.config.RobotState
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Data.Config.RobotConfig

@TeleOp(name = "Test - Gate and Hood", group = "Test")
class GateHoodTestOpMode : NextFTCOpMode() {

    init {
        addComponents(
            SubsystemComponent(Gate, Hood),
            BulkReadComponent,
            BindingsComponent
        )
    }

    override fun onInit() {
        telemetry.addLine("=== GATE & HOOD TEST ===")
        telemetry.addLine("CONTROLS:")
        telemetry.addLine("  Y - Open Gate")
        telemetry.addLine("  A - Close Gate")
        telemetry.addLine()
        telemetry.addLine("  D-Pad UP - Move Hood Up (Step)")
        telemetry.addLine("  D-Pad DOWN - Move Hood Down (Step)")
        telemetry.addLine("  Back - Reset Hood to 0.0")
        telemetry.update()
    }

    override fun onUpdate() {
        // ==================== GATE CONTROLS ====================
        if (gamepad1.y) {
            Gate.open.schedule()
        }
        if (gamepad1.a) {
            Gate.close.schedule()
        }

        // ==================== HOOD CONTROLS ====================
        // Using schedule() for your commands
        if (gamepad1.dpad_up) {
            Hood.moveUp.schedule()
        }
        if (gamepad1.dpad_down) {
            Hood.moveDown.schedule()
        }
        if (gamepad1.back) {
            Hood.reset.schedule()
        }

        // ==================== TELEMETRY ====================
        ActiveOpMode.telemetry.run {
            addLine("=== GATE STATUS ===")
            addData("Position", if (Gate.isOpen()) "OPEN" else "CLOSED")
            addData("Raw Target", RobotConfig.GateConfig.let {
                if (Gate.isOpen()) it.openPosition else it.closedPosition
            })

            addLine()
            addLine("=== HOOD STATUS ===")
            addData("Current Pos", "%.3f".format(RobotState.hoodPosition))
            addData("Min/Max", "%.2f / %.2f".format(
                RobotConfig.HoodConfig.MIN_POSITION,
                RobotConfig.HoodConfig.MAX_POSITION
            ))

            update()
        }
    }
}