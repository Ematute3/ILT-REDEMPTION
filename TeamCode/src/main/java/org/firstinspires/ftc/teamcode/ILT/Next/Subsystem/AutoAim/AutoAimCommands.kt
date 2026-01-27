package org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.AutoAim

import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.groups.ParallelGroup
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.commands.utility.LambdaCommand
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Data.Enums.IntakeState
import org.firstinspires.ftc.teamcode.robot.data.config.RobotState

import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Shooter.FlyWheel
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Shooter.Hood
import kotlin.time.Duration.Companion.milliseconds
import kotlin.time.Duration.Companion.seconds

/**
 * Custom WaitUntil (good pattern; matches NextFTC style)
 */
class WaitUntil(private val condition: () -> Boolean) : Command() {
    override val isDone: Boolean
        get() = condition()

    // No start/update/stop needed unless you want logging
}

/**
 * Pre-built commands for auto-aiming system
 */
object AutoAimCommands {

    // ── BASIC AIM COMMANDS ───────────────────────────────────────────────

    val calculateAndApply = InstantCommand {
        AutoAimCalculator.applyToSubsystems()
    }

    val aimFromOdometry = InstantCommand {
        AutoAimCalculator.calculateFromOdometry()?.let { params ->
            AutoAimCalculator.applyParameters(params)
        }
    }

    val aimFromLimelight = InstantCommand {
        AutoAimCalculator.calculateFromLimelight()?.let { params ->
            AutoAimCalculator.applyParameters(params)
        }
    }

    val aimBestAvailable = InstantCommand {
        AutoAimCalculator.applyToSubsystems()
    }

    // ── CONTINUOUS AIM COMMAND ───────────────────────────────────────────

    val continuousAim = LambdaCommand()
        .setUpdate {
            val params = AutoAimCalculator.calculateBestAvailable()

            // Only update if distance changed significantly (reduce jitter)
            if (kotlin.math.abs(params.distance - lastDistance) > 3.0) {
                AutoAimCalculator.applyParameters(params)
                lastDistance = params.distance
            }
        }
        .named("Continuous Auto Aim")
    // No .setIsDone() → runs until interrupted / cancelled

    private var lastDistance: Double = 0.0   // moved outside since lambda captures

    // ── COMPLETE SHOOTING SEQUENCES ──────────────────────────────────────

    fun createAutoShootSequence(): Command = SequentialGroup(
        InstantCommand { AutoAimCalculator.applyToSubsystems() },

        InstantCommand { FlyWheel.setTargetVelocity(AutoAimCalculator.getFlywheelVelocity()) },

        WaitUntil { RobotState.flywheelAtSpeed },
        Delay(2.seconds),

        InstantCommand { AutoAimCalculator.applyToSubsystems() },

        WaitUntil { RobotState.turretAligned && RobotState.flywheelAtSpeed },
        Delay(1.seconds),

        InstantCommand {
            RobotState.intakeState = IntakeState.FEEDING
        },

        Delay(500.milliseconds),

        InstantCommand {
            FlyWheel.setTargetVelocity(0.0)
            RobotState.intakeState = IntakeState.STOPPED
        }
    )

    fun createPrepareToShootSequence(): Command = SequentialGroup(
        InstantCommand { AutoAimCalculator.applyToSubsystems() },

        ParallelGroup(
            InstantCommand { FlyWheel.setTargetVelocity(AutoAimCalculator.getFlywheelVelocity()) },
            continuousAim
        ),

        WaitUntil { RobotState.flywheelAtSpeed && RobotState.turretAligned },
        Delay(2.seconds)
    )

    fun createFireSequence(): Command = SequentialGroup(
        InstantCommand {
            RobotState.intakeState = IntakeState.FEEDING
        },

        Delay(500.milliseconds),

        InstantCommand {
            RobotState.intakeState = IntakeState.STOPPED
        }
    )

    // ── TESTING / TUNING HELPERS ─────────────────────────────────────────

    fun testAtDistance(distance: Double): Command = InstantCommand {
        val params = AutoAimCalculator.calculateFromDistance(distance)
        AutoAimCalculator.applyParameters(params)
    }

    fun createDistanceTestSequence(): Command {
        val distances = AutoAimCalculator.getValidDistances()
        val commands = distances.map { distance ->
            SequentialGroup(
                InstantCommand {
                    val params = AutoAimCalculator.calculateFromDistance(distance.toDouble())
                    AutoAimCalculator.applyParameters(params)
                },
                Delay(2.seconds)
            )
        }
        return SequentialGroup(*commands.toTypedArray())
    }

    fun adjustHoodOffset(delta: Double): Command = InstantCommand {
        AutoAimCalculator.hoodOffset += delta
        AutoAimCalculator.applyToSubsystems()
    }

    fun adjustVelocityOffset(delta: Double): Command = InstantCommand {
        AutoAimCalculator.velocityOffset += delta
        AutoAimCalculator.applyToSubsystems()
    }

    val resetToDefaults = InstantCommand {
        Hood.setPosition(0.5)
        FlyWheel.setTargetVelocity(0.0)
    }

    val printDebugInfo = InstantCommand {
        println(AutoAimCalculator.getDebugInfo())
    }
}