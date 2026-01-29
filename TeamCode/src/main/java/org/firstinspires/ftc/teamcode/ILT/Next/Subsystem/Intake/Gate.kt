package org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Intake

import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode
import dev.nextftc.hardware.impl.ServoEx
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Data.Config.RobotConfig


/**
 * Gate subsystem for controlling ball flow to shooter.
 */
object Gate : Subsystem {

    private var servo = ServoEx(RobotConfig.Hardware.GATE_SERVO)
    private var position = RobotConfig.GateConfig.openPosition


    override fun initialize() {

    }

    override fun periodic() {

        servo.position = position
    }

    /**
     * Set gate position (0.0 = open, 1.0 = closed, typically).
     */
    fun setPosition(newPosition: Double) {
        position = newPosition.coerceIn(0.0, 1.0)
    }

    /**
     * Check if gate is open.
     */
    fun isOpen(): Boolean = position == RobotConfig.GateConfig.openPosition

    // ==================== COMMANDS ====================

    val open = InstantCommand {
        position = RobotConfig.GateConfig.openPosition
    }

    val close = InstantCommand {
        position = RobotConfig.GateConfig.closedPosition
    }
}