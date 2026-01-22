package org.firstinspires.ftc.teamcode.robot.subsystems.shooter

import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode
import dev.nextftc.hardware.impl.ServoEx
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Data.Config.RobotConfig
import org.firstinspires.ftc.teamcode.robot.data.config.RobotState

/**
 * Hood subsystem for adjusting shot angle.
 */
object Hood : Subsystem {

    private lateinit var servo: ServoEx
    private var isInitialized = false

    override fun initialize() {
        try {
            servo = ServoEx(RobotConfig.Hardware.HOOD_SERVO)
            isInitialized = true
        } catch (e: Exception) {
            ActiveOpMode.telemetry.addData("Hood Error", e.message)
            isInitialized = false
        }
    }

    override fun periodic() {
        if (!isInitialized) return

        // Apply clamped position
        servo.position = RobotState.hoodPosition.coerceIn(
            RobotConfig.HoodConfig.MIN_POSITION,
            RobotConfig.HoodConfig.MAX_POSITION
        )

        ActiveOpMode.telemetry.addData("Hood Position", "%.2f".format(RobotState.hoodPosition))
    }

    /**
     * Set hood position (0.0 to 1.0).
     */
    fun setPosition(position: Double) {
        RobotState.hoodPosition = position.coerceIn(
            RobotConfig.HoodConfig.MIN_POSITION,
            RobotConfig.HoodConfig.MAX_POSITION
        )
    }

    /**
     * Get current hood position.
     */
    fun getPosition(): Double = RobotState.hoodPosition

    // ==================== COMMANDS ====================

    val moveUp = InstantCommand {
        RobotState.hoodPosition = (RobotState.hoodPosition - RobotConfig.HoodConfig.adjustStep)
            .coerceIn(RobotConfig.HoodConfig.MIN_POSITION, RobotConfig.HoodConfig.MAX_POSITION)
    }

    val moveDown = InstantCommand {
        RobotState.hoodPosition = (RobotState.hoodPosition + RobotConfig.HoodConfig.adjustStep)
            .coerceIn(RobotConfig.HoodConfig.MIN_POSITION, RobotConfig.HoodConfig.MAX_POSITION)
    }

    val reset = InstantCommand {
        RobotState.hoodPosition = 0.0
    }
}