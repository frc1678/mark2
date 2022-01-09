package com.team1678.frc2021.auto;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.spline.Spline;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.nio.file.Path;
import java.util.Arrays;

/* FROM FRC 4277 */

public class WaypointReader {
    private static final double PATHWEAVER_Y_OFFSET = 8.21055; // See https://www.desmos.com/calculator/0lqfdhxrmj

    /**
     * Get control vector list from path file
     * @param pathName Specify the {THIS} in src/main/deploy/waypoints/{THIS}.path
     * @return control vectors from file
     */
    public static TrajectoryGenerator.ControlVectorList getControlVectors(Path path) throws IOException {

        TrajectoryGenerator.ControlVectorList controlVectors = new TrajectoryGenerator.ControlVectorList();

        try (BufferedReader reader = new BufferedReader(new FileReader(path.toFile()))){
            boolean skippedFirst = false;
            String line = reader.readLine();
            while (line != null) {
                if (!skippedFirst || !line.contains(",")) {
                    skippedFirst = true;
                    line = reader.readLine();
                    continue;
                }
                String[] split = line.split(",");
                controlVectors.add(new Spline.ControlVector(
                        new double[]{Double.parseDouble(split[0]), Double.parseDouble(split[2]), 0},
                        new double[]{Double.parseDouble(split[1]) + PATHWEAVER_Y_OFFSET, Double.parseDouble(split[3]), 0}));

                line = reader.readLine();
            }
        }

        // Debug print out of arrays in vectors
        for (Spline.ControlVector vector : controlVectors) {
            System.out.println(Arrays.toString(vector.x));
            System.out.println(Arrays.toString(vector.y));
            System.out.println("===");
        }

        return controlVectors;
    }
}
