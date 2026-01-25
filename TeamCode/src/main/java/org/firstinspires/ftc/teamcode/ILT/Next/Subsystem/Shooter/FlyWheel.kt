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
 * FlyWheel subsystem - uses NextFTC controlSystem for velocity control.
 *
 * Based on NextControl documentation example.
 */
object FlyWheel : Subsystem {

    // ==================== HARDWARE ====================
    val fly1 = MotorEx(RobotConfig.Hardware.FLYWHEEL_1).floatMode()
    val fly2 = MotorEx(RobotConfig.Hardware.FLYWHEEL_2).reversed().floatMode()

    // ==================== CONTROL SYSTEM ====================
    var controller = controlSystem {
        velPid(RobotConfig.FlywheelConfig.pid)
        basicFF(RobotConfig.FlywheelConfig.feedforward)
    }

    // ==================== STATE ====================
    @JvmField
    var targetVelocity = 0.0

    var motorRpm: Double = 0.0

    // ==================== INITIALIZATION ====================
    override fun initialize() {
        // Safety: Flywheels should FLOAT to stop, not BRAKE


        // Initialize goal to stopped
        controller.goal = KineticState(0.0, 0.0)
    }

    // ==================== PERIODIC ====================
    override fun periodic() {
        // 1. Update sensor data
        motorRpm = fly1.velocity * 60.0 / RobotConfig.FlywheelConfig.MOTOR_TICKS_PER_REV

        // 2. Calculate power using the EXACT pattern from NextControl docs:
        // controller.calculate(KineticState(position, velocity))
        val power = controller.calculate(
            KineticState(
                fly1.motor.currentPosition.toDouble(),
                targetVelocity
            )
        )

        // 3. Apply power to motors
        fly1.power = power
        fly2.power = power

        // 4. Update Global Robot State
        RobotState.flywheelVelocity = fly1.velocity
        RobotState.flywheelAtSpeed = isAtTargetVelocity()
        RobotState.targetFlywheelVelocity = targetVelocity

        // 5. Telemetry
        ActiveOpMode.telemetry.run {
            addData("--- FlyWheel ---", "")
            addData("Target Vel", "%.0f".format(targetVelocity))
            addData("Actual Vel", "%.0f".format(fly1.velocity))
            addData("RPM", "%.0f".format(motorRpm))
            addData("Power", "%.2f".format(power))
            addData("At Speed", isAtTargetVelocity())
        }
    }

    // ==================== HELPER FUNCTIONS ====================

    fun isAtTargetVelocity(): Boolean {
        return abs(fly1.velocity - targetVelocity) < RobotConfig.FlywheelConfig.velocityTolerance
    }

    fun getCurrentVelocity(): Double = fly1.velocity

    fun getRPM(): Double = motorRpm

    // ==================== CONTROL FUNCTIONS ====================

    /** Set target velocity and update controller goal */
    fun setTargetVelocity(velocity: Double) {
        targetVelocity = velocity
        controller.goal = KineticState(0.0, velocity)
    }

    /** Spin up to specified velocity */
    fun spinUp(velocity: Double) {
        setTargetVelocity(velocity)
    }

    /** Stop the flywheel */
    fun stop() {
        targetVelocity = 0.0

    }

    // ==================== COMMANDS ====================

    /** Command to spin at full power (bypasses PID) */
    val spinFull = InstantCommand {
        fly1.power = 1.0
        fly2.power = 1.0
    }
    val spin = InstantCommand{
        targetVelocity = 1500.0

    }
    val stop = InstantCommand{
        targetVelocity = 0.0
    }

    /** Command to stop motors directly */
    val stopMotors = InstantCommand {
        fly1.power = 0.0
        fly2.power = 0.0
    }

    /** Command to reverse (clear jams) */
    val reverse = InstantCommand {
        fly1.power = -0.5
        fly2.power = -0.5
    }
}