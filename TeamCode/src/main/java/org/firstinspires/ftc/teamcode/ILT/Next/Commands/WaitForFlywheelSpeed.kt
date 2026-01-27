package org.firstinspires.ftc.teamcode.ILT.Next.Commands

import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.ftc.ActiveOpMode
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Data.Aimbot.AimbotTable
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Data.Config.RobotConfig
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Intake.Intake
import org.firstinspires.ftc.teamcode.robot.data.config.RobotState
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Intake.Gate
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Shooter.FlyWheel
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Shooter.Hood
import kotlin.time.Duration.Companion.milliseconds

/**
 * COMMAND: Waits for flywheel to reach target RPM
 */
class WaitForFlywheelSpeed(
    private val timeoutMs: Long = RobotConfig.Timing.FLYWHEEL_SPINUP_TIMEOUT_MS
) : Command() {
    private var startTime = 0L

    override val isDone: Boolean
        get() = FlyWheel.isAtTargetVelocity() || (System.currentTimeMillis() - startTime) > timeoutMs

    override fun start() { startTime = System.currentTimeMillis() }

    override fun update() {
        ActiveOpMode.telemetry.addData("Flywheel Wait", "%.0f RPM".format(FlyWheel.getCurrentVelocity()))
    }
}

/**
 * OBJECT: Collection of firing sequences
 */
