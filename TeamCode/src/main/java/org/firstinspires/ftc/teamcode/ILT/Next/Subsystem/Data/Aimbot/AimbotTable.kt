package org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Data.Aimbot



/**
 * Lookup table for hood position and flywheel velocity based on distance.
 * Values are tuned empirically during testing.
 */
object AimbotTable {

    /**
     * Distance-indexed lookup table.
     * Each entry: distance (inches) -> Pair(hoodPosition, flywheelVelocity)
     */
    private val lookupTable: Map<Int, Pair<Double, Double>> = mapOf(
        12 to Pair(0.81, 835.0),
        24 to Pair(0.93, 862.0),
        36 to Pair(0.71, 844.0),
        48 to Pair(0.60, 848.0),
        60 to Pair(0.62, 908.0),
        72 to Pair(0.65, 1025.0),
        84 to Pair(0.70, 1165.0),
        96 to Pair(0.70, 1230.0),
        108 to Pair(0.42, 1070.0),
        120 to Pair(0.43, 1112.0),
        132 to Pair(0.44, 1150.0),
        144 to Pair(0.45, 1250.0)
    )

    // Sorted distances for interpolation
    private val distances = lookupTable.keys.sorted()

    /**
     * Get aim values for a given distance.
     * Returns Pair(hoodPosition, flywheelVelocity).
     * * Uses linear interpolation between known values.
     * Fixed: Clamps distance to range to prevent null returns.
     */
    fun getValues(distance: Double): Pair<Double, Double> {
        // Safety: Clamp the distance so it never falls outside our defined keys (12 to 144)
        val clampedDist = distance.coerceIn(distances.first().toDouble(), distances.last().toDouble())

        // Find surrounding distances for interpolation
        var lowerDist = distances.first()
        var upperDist = distances.last()

        for (d in distances) {
            if (d <= clampedDist) lowerDist = d
            if (d >= clampedDist) {
                upperDist = d
                break
            }
        }

        // Exact match or clamped to edge
        if (lowerDist == upperDist) {
            return lookupTable[lowerDist]!!
        }

        // Interpolate
        val lowerVals = lookupTable[lowerDist]!!
        val upperVals = lookupTable[upperDist]!!

        val t = (clampedDist - lowerDist) / (upperDist - lowerDist)

        val hood = lowerVals.first + t * (upperVals.first - lowerVals.first)
        val velocity = lowerVals.second + t * (upperVals.second - lowerVals.second)

        return Pair(hood, velocity)
    }

    /**
     * Get hood position for distance (with optional offset applied)
     */
    fun getHoodPosition(distance: Double, offset: Double = 0.06): Double {
        return (getValues(distance).first + offset).coerceIn(0.0, 1.0)
    }

    /**
     * Get flywheel velocity for distance (with optional offset applied)
     */
    fun getFlywheelVelocity(distance: Double, offset: Double = 100.0): Double {
        return getValues(distance).second + offset
    }

    /**
     * Snap to nearest valid distance (multiples of 12)
     * Used for manual operator adjustments.
     */
    fun snapToValidDistance(distance: Int): Int {
        val snapped = (distance / 12) * 12
        return snapped.coerceIn(12, 144)
    }

    /**
     * Get all valid distances for UI/debugging
     */
    fun getValidDistances(): List<Int> = distances
}