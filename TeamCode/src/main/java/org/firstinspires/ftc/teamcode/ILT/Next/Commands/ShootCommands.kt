import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.commands.utility.InstantCommand
import org.firstinspires.ftc.teamcode.ILT.Next.Commands.WaitForFlywheelSpeed
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Data.Aimbot.AimbotTable
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Data.Config.RobotConfig
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Intake.Intake
import org.firstinspires.ftc.teamcode.robot.data.config.RobotState
import org.firstinspires.ftc.teamcode.robot.subsystems.intake.Gate
import org.firstinspires.ftc.teamcode.robot.subsystems.shooter.FlyWheel
import org.firstinspires.ftc.teamcode.robot.subsystems.shooter.Hood
import kotlin.time.Duration.Companion.milliseconds

object ShootCommands {
    val shoot: Command
        get() = SequentialGroup(
            FlyWheel.spin,
            WaitForFlywheelSpeed(),
            Gate.open,                         // Physically open the path
            Intake.feed,                       // Push ball into wheel
            Delay(RobotConfig.Timing.FEED_DURATION_MS.milliseconds),
            Gate.close,                        // Reset for next shot
            Intake.stop
        )

    val fullAutoShoot: Command
        get() = SequentialGroup(
            InstantCommand {
                val dist = if (RobotState.poseValid) RobotState.distanceToGoalOdometry else 72.0
                val values = AimbotTable.getValues(dist)
                Hood.setPosition(values.first + 0.06)
                FlyWheel.setTargetVelocity(values.second + 100)
            },
            shoot
        )

    val limelightShoot: Command
        get() = SequentialGroup(
            InstantCommand {
                val distance = RobotState.distanceToGoalLimelight ?: 72.0
                val values = AimbotTable.getValues(distance)
                Hood.setPosition(values.first + 0.06)
                FlyWheel.setTargetVelocity(values.second + 100)
            },
            shoot
        )

    val stopAll: Command
        get() = SequentialGroup(FlyWheel.stop, Intake.stop, Gate.close)
}