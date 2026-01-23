package org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Intake

import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode
import dev.nextftc.hardware.controllable.MotorGroup
import dev.nextftc.hardware.impl.MotorEx
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Data.Config.RobotConfig

import org.firstinspires.ftc.teamcode.robot.data.config.RobotState
import org.firstinspires.ftc.teamcode.robot.data.enums.IntakeState



/**
 * Intake subsystem for collecting and feeding balls.
 */
object Intake : Subsystem {

    private lateinit var intakeMotor: MotorEx



    private var power = 0.0
    private var isInitialized = false

    override fun initialize() {
        try {
            intakeMotor = MotorEx(RobotConfig.Hardware.INTAKE)

            isInitialized = true
        } catch (e: Exception) {
            ActiveOpMode.telemetry.addData("Intake Error", e.message)
            isInitialized = false
        }
    }

    override fun periodic() {
        if (!isInitialized) return

        intakeMotor.power = power

        ActiveOpMode.telemetry.run {
            addData("Intake State", RobotState.intakeState)
            addData("Intake Power", "%.2f".format(power))
        }
    }

    /**
     * Set intake power directly.
     */
    fun setPower(newPower: Double) {
        power = newPower
    }

    /**
     * Check if intake is currently running.
     */
    fun isRunning(): Boolean = RobotState.intakeState != IntakeState.STOPPED

    // ==================== COMMANDS ====================

    val run = InstantCommand {
        power = RobotConfig.IntakeConfig.intakePower
        RobotState.intakeState = IntakeState.INTAKING
    }

    val reverse = InstantCommand {
        power = RobotConfig.IntakeConfig.ejectPower
        RobotState.intakeState = IntakeState.EJECTING
    }

    val reverseSlow = InstantCommand {
        power = RobotConfig.IntakeConfig.ejectSlowPower
        RobotState.intakeState = IntakeState.EJECTING
    }

    val feed = InstantCommand {
        power = RobotConfig.IntakeConfig.feedPower
        RobotState.intakeState = IntakeState.FEEDING
    }

    val stop = InstantCommand {
        power = 0.0
        RobotState.intakeState = IntakeState.STOPPED
    }
}