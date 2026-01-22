package org.firstinspires.ftc.teamcode.ILT.Next.Commands

import dev.nextftc.core.commands.Command
import dev.nextftc.ftc.ActiveOpMode
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Data.Config.RobotConfig

import org.firstinspires.ftc.teamcode.robot.subsystems.shooter.FlyWheel

/**
 * Non-blocking command that waits for the flywheel to reach target velocity.
 * Finishes when flywheel is at speed OR timeout is exceeded.
 */
class WaitForFlywheelSpeed(
    private val timeoutMs: Long = RobotConfig.Timing.FLYWHEEL_SPINUP_TIMEOUT_MS
) : Command() {

    private var startTime = 0L

    override val isDone: Boolean
        get() {
            val elapsed = System.currentTimeMillis() - startTime
            return FlyWheel.isAtTargetVelocity() || elapsed > timeoutMs
        }

    override fun start() {
        startTime = System.currentTimeMillis()
    }

    override fun update() {
        val elapsed = System.currentTimeMillis() - startTime
        ActiveOpMode.telemetry.run {
            addData("Flywheel Wait", "%.1fs / %.1fs".format(elapsed / 1000.0, timeoutMs / 1000.0))
            addData("Current", "%.0f".format(FlyWheel.getVelocity()))
            addData("At Speed", FlyWheel.isAtTargetVelocity())
        }
    }

    override fun stop(interrupted: Boolean) {
        if (interrupted) {
            ActiveOpMode.telemetry.addData("Flywheel Wait", "Interrupted")
        } else {
            ActiveOpMode.telemetry.addData("Flywheel Wait",
                if (FlyWheel.isAtTargetVelocity()) "Ready!" else "Timed out")
        }
    }
}