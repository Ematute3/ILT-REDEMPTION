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
import org.firstinspires.ftc.teamcode.robot.subsystems.shooter.FlyWheel
import org.firstinspires.ftc.teamcode.robot.subsystems.shooter.Hood
import kotlin.time.Duration.Companion.milliseconds


/**
 * Shooting command sequences.
 * These are pre-built command groups for common shooting operations.
 */
object ShootCommands {

    /**
     * Simple shoot sequence - assumes aiming is already done.
     * Spins up, waits for speed, feeds, stops.
     */
    val simpleShoot: Command
        get() = SequentialGroup(
            FlyWheel.spin,
            WaitForFlywheelSpeed(),
            Intake.feed,
            Delay(RobotConfig.Timing.FEED_DURATION_MS.milliseconds),
            FlyWheel.stop,
            Intake.stop
        )

    /**
     * Full auto shoot sequence.
     * Sets aim values, spins up, waits, feeds, stops.
     */
    val fullAutoShoot: Command
        get() = SequentialGroup(
            // Set aim values based on odometry distance
            InstantCommand {
                if (!RobotState.poseValid) {
                    ActiveOpMode.telemetry.addData("Auto Shoot", "No pose!")
                    return@InstantCommand
                }

                val values = AimbotTable.getValues(RobotState.distanceToGoalOdometry)
                if (values != null) {
                    Hood.setPosition(values.first + 0.06)
                    FlyWheel.setTargetVelocity(values.second + 100)
                }
            },

            // Spin up
            FlyWheel.spin,

            // Wait for speed
            WaitForFlywheelSpeed(),

            // Feed ball
            Intake.feed,
            Delay(RobotConfig.Timing.FEED_DURATION_MS.milliseconds),

            // Stop
            FlyWheel.stop,
            Intake.stop
        )

    /**
     * Limelight-based auto shoot.
     * Uses Limelight distance for aiming.
     */
    val limelightShoot: Command
        get() = SequentialGroup(
            // Set aim values based on Limelight distance
            InstantCommand {
                val distance = RobotState.distanceToGoalLimelight
                if (distance == null) {
                    ActiveOpMode.telemetry.addData("LL Shoot", "No target!")
                    return@InstantCommand
                }

                val values = AimbotTable.getValues(distance)
                if (values != null) {
                    Hood.setPosition(values.first + 0.06)
                    FlyWheel.setTargetVelocity(values.second + 100)
                }
            },

            FlyWheel.spin,
            WaitForFlywheelSpeed(),
            Intake.feed,
            Delay(RobotConfig.Timing.FEED_DURATION_MS.milliseconds),
            FlyWheel.stop,
            Intake.stop
        )

    /**
     * Manual distance shoot - uses a specified distance.
     */
    fun shootAtDistance(distanceInches: Int): Command {
        return SequentialGroup(
            InstantCommand {
                val snapped = AimbotTable.snapToValidDistance(distanceInches)
                val values = AimbotTable.getValues(snapped.toDouble())
                if (values != null) {
                    Hood.setPosition(values.first)
                    FlyWheel.setTargetVelocity(values.second)
                    ActiveOpMode.telemetry.addData("Manual Shoot", "Distance: $snapped\"")
                }
            },

            FlyWheel.spin,
            WaitForFlywheelSpeed(),
            Intake.feed,
            Delay(RobotConfig.Timing.FEED_DURATION_MS.milliseconds),
            FlyWheel.stop,
            Intake.stop
        )
    }

    /**
     * Emergency stop - stops everything immediately.
     */
    val emergencyStop: Command
        get() = SequentialGroup(
            FlyWheel.stop,
            Intake.stop
        )
}