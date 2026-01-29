package org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Shooter

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

    private var servo = ServoEx(RobotConfig.Hardware.HOOD_SERVO)


    override fun initialize() {

    }

    override fun periodic() {
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
    val up = InstantCommand{
        servo.position = 1.0
    }
    val down = InstantCommand{
        servo.position = 0.0
    }

    val reset = InstantCommand {
        RobotState.hoodPosition = 0.0
    }
}