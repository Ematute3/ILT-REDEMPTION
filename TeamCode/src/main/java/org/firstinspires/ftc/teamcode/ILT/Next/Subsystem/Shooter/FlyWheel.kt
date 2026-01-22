package org.firstinspires.ftc.teamcode.robot.subsystems.shooter

import dev.nextftc.control.KineticState
import dev.nextftc.control.builder.controlSystem
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode
import dev.nextftc.hardware.impl.MotorEx
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Data.Config.RobotConfig

import org.firstinspires.ftc.teamcode.robot.data.config.RobotState
import kotlin.math.abs

/**
 * Flywheel subsystem for ball shooting.
 * Uses velocity PID control with feedforward.
 */
object FlyWheel : Subsystem {

    private lateinit var motor1: MotorEx
    private lateinit var motor2: MotorEx

    private var controller = controlSystem {
        velPid(RobotConfig.FlywheelConfig.pid)
        basicFF(RobotConfig.FlywheelConfig.feedforward)
    }

    private var targetVelocity = 0.0
    private var manualMode = false
    private var isInitialized = false

    override fun initialize() {
        try {
            motor1 = MotorEx(RobotConfig.Hardware.FLYWHEEL_1)
            motor2 = MotorEx(RobotConfig.Hardware.FLYWHEEL_2).reversed()
            isInitialized = true
        } catch (e: Exception) {
            ActiveOpMode.telemetry.addData("FlyWheel Error", e.message)
            isInitialized = false
        }
    }

    override fun periodic() {
        if (!isInitialized) return

        // Update state
        RobotState.flywheelVelocity = motor1.velocity
        RobotState.targetFlywheelVelocity = targetVelocity
        RobotState.flywheelAtSpeed = isAtTargetVelocity()

        // Run control loop (unless in manual mode)
        if (!manualMode) {
            val currentState = KineticState(motor1.currentPosition, motor1.velocity)

            controller.goal = if (RobotState.flywheelOn) {
                KineticState(0.0, targetVelocity)
            } else {
                KineticState(0.0, 0.0)
            }

            val power = controller.calculate(currentState)
            motor1.power = power
            motor2.power = power
        }

        // Telemetry
        ActiveOpMode.telemetry.run {
            addData("Flywheel On", RobotState.flywheelOn)
            addData("Target Velocity", "%.0f".format(targetVelocity))
            addData("Current Velocity", "%.0f".format(motor1.velocity))
            addData("At Speed", RobotState.flywheelAtSpeed)
            addData("RPM", "%.0f".format(motor1.velocity * 60.0 / RobotConfig.FlywheelConfig.MOTOR_TICKS_PER_REV))
        }
    }

    /**
     * Check if flywheel is at target velocity.
     */
    fun isAtTargetVelocity(): Boolean {
        return RobotState.flywheelOn &&
                abs(motor1.velocity - targetVelocity) < RobotConfig.FlywheelConfig.velocityTolerance
    }

    /**
     * Set target velocity (ticks/sec).
     */
    fun setTargetVelocity(velocity: Double) {
        targetVelocity = velocity
    }

    /**
     * Get current velocity (ticks/sec).
     */
    fun getVelocity(): Double = if (isInitialized) motor1.velocity else 0.0

    /**
     * Get current RPM.
     */
    fun getRPM(): Double = getVelocity() * 60.0 / RobotConfig.FlywheelConfig.MOTOR_TICKS_PER_REV

    // ==================== COMMANDS ====================

    val spin = InstantCommand {
        manualMode = false
        RobotState.flywheelOn = true
    }

    val stop = InstantCommand {
        manualMode = false
        RobotState.flywheelOn = false
    }

    val backOutSlow = InstantCommand {
        manualMode = true
        RobotState.flywheelOn = false
        if (isInitialized) {
            motor1.power = -0.5
            motor2.power = -0.5
        }
    }

    val backOut = InstantCommand {
        manualMode = true
        RobotState.flywheelOn = false
        if (isInitialized) {
            motor1.power = -1.0
            motor2.power = -1.0
        }
    }
}