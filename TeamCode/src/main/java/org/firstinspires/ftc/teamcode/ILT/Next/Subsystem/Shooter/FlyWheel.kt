package org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Shooter

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

    private var lastPidKp = RobotConfig.FlywheelConfig.pid.kP
    private var lastPidKi = RobotConfig.FlywheelConfig.pid.kI
    private var lastPidKd = RobotConfig.FlywheelConfig.pid.kD
    private var lastFfKv = RobotConfig.FlywheelConfig.feedforward.kV
    private var lastFfKs = RobotConfig.FlywheelConfig.feedforward.kS
    private var lastFfKa = RobotConfig.FlywheelConfig.feedforward.kA

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
        updateControllerFromConfig()

        motorRpm = fly1.velocity * 60.0 / RobotConfig.FlywheelConfig.MOTOR_TICKS_PER_REV

        // Calculate power using current config (PID/FF from Panels)
        // controller.calculate(KineticState(position, velocity))
        // Use actual velocity, not target velocity!
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

    /** Rebuild controller when PID/FF change on Panels so edits take effect live. */
    private fun updateControllerFromConfig() {
        val pid = RobotConfig.FlywheelConfig.pid
        val ff = RobotConfig.FlywheelConfig.feedforward
        val pidChanged = pid.kP != lastPidKp || pid.kI != lastPidKi || pid.kD != lastPidKd
        val ffChanged = ff.kV != lastFfKv || ff.kS != lastFfKs || ff.kA != lastFfKa
        if (pidChanged || ffChanged) {
            controller = controlSystem {
                velPid(pid)
                basicFF(ff)
            }
            controller.goal = KineticState(0.0, targetVelocity)
            lastPidKp = pid.kP
            lastPidKi = pid.kI
            lastPidKd = pid.kD
            lastFfKv = ff.kV
            lastFfKs = ff.kS
            lastFfKa = ff.kA
        }
    }

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


    // ==================== COMMANDS ====================

    /** Command to spin at full power (bypasses PID) */

    val spin = InstantCommand{
        targetVelocity = 200.0

    }
    val stop = InstantCommand{
        targetVelocity = 0.0
    }

    val reverse = InstantCommand {
        targetVelocity = -100.0
    }
}