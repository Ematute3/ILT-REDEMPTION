package org.firstinspires.ftc.teamcode.ILT.Next.TestOp

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Data.Config.RobotConfig
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Shooter.Turret
import kotlin.math.abs

@TeleOp(name = "Test - Turret", group = "Test")
class TurretTestOpMode : NextFTCOpMode() {

    init {
        addComponents(
            SubsystemComponent(Turret),
            BulkReadComponent
        )
    }

    private var targetAngleDegrees = 0.0

    override fun onInit() {
        telemetry.addLine("=== TURRET TEST ===")
        telemetry.addLine("Align turret FORWARD before pressing START")
        telemetry.update()
    }

    override fun onUpdate() {
        // 1. ==================== MANUAL CONTROL ====================
        val manualInput = -gamepad1.left_stick_x.toDouble()

        if (abs(manualInput) > 0.05) {
            // Update the subsystem state and power
            Turret.currentState = Turret.State.MANUAL
            Turret.manualPower = manualInput * RobotConfig.TurretConfig.manualPowerFast
        }

        // 2. ==================== PRESET POSITIONS ====================
        else if (gamepad1.a || gamepad1.b || gamepad1.x || gamepad1.y) {
            Turret.currentState = Turret.State.ODOMETRY // Using PID logic

            if (gamepad1.a) targetAngleDegrees = 0.0
            if (gamepad1.b) targetAngleDegrees = 45.0
            if (gamepad1.x) targetAngleDegrees = -45.0
            if (gamepad1.y) targetAngleDegrees = 90.0

            // Tell the Turret controller where to go (in Radians)
            Turret.goToYawDegrees(targetAngleDegrees)
        }

        // 3. ==================== IDLE / STOP ====================
        else if (Turret.currentState == Turret.State.MANUAL) {
            // If we were in manual and let go of the stick, stop.
            Turret.currentState = Turret.State.IDLE
            Turret.manualPower = 0.0
        }

        // 4. ==================== ZERO ENCODER ====================
        if (gamepad1.right_bumper) {
            Turret.resetEncoderToZero()
        }

        // 5. ==================== TUNING ADJUSTMENTS ====================
        // Note: These update the JvmFields in the Turret object directly
        if (gamepad1.dpad_up) Turret.kP_limelight += 0.0001
        if (gamepad1.dpad_down) Turret.kP_limelight = (Turret.kP_limelight - 0.0001).coerceAtLeast(0.0)

        if (gamepad1.dpad_right) Turret.maxPower = (Turret.maxPower + 0.01).coerceAtMost(1.0)
        if (gamepad1.dpad_left) Turret.maxPower = (Turret.maxPower - 0.01).coerceAtLeast(0.1)

        // 6. ==================== TELEMETRY ====================
        telemetry.run {
            addLine("=== TURRET TEST ===")
            addData("State", Turret.currentState)
            addData("Current Angle", "%.1f°".format(Turret.getYawDegrees()))
            addData("Target Angle", "%.1f°".format(targetAngleDegrees))
            addData("Raw Ticks", Turret.getRawTicks())
            addData("Power", "%.2f".format(Turret.manualPower)) // Shows power sent to manual mode
            addLine("--- Tuning ---")
            addData("kP_LL", "%.4f".format(Turret.kP_limelight))
            addData("MaxPower", "%.2f".format(Turret.maxPower))
            update()
        }
    }
}