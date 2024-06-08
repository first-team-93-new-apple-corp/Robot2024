package frc.robot.subsystems.Swerve;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.util.HashSet;
import java.util.Set;

import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain.SwerveDriveState;
import com.pathplanner.lib.pathfinding.LocalADStar.GridPosition;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Filesystem;

public class CollisionDeteaction {

    private double fieldLength = 16.54;
    private double fieldWidth = 8.02;

    private double nodeSize = 0.2;
    private int nodesX = (int) Math.ceil(fieldLength / nodeSize);
    private int nodesY = (int) Math.ceil(fieldWidth / nodeSize);

    private GridPosition currentGridPosition;
    private final Set<GridPosition> staticObstacles = new HashSet<>();

    private double loss = 0;

    public CollisionDeteaction() {
        // we use the same NavGrid as path planner, and I am doing the nav grid part very similar to them. basicaly copied the code, but I did write it becuase I wanted to understand it
        File navGridFile = new File(Filesystem.getDeployDirectory(), "pathplanner/navgrid.json");
        if (navGridFile.exists()) {
            try (BufferedReader br = new BufferedReader(new FileReader(navGridFile))) {
                StringBuilder fileContentBuilder = new StringBuilder();
                String line;
                while ((line = br.readLine()) != null) {
                    fileContentBuilder.append(line);
                }

                String fileContent = fileContentBuilder.toString();
                JSONObject json = (JSONObject) new JSONParser().parse(fileContent);

                nodeSize = ((Number) json.get("nodeSizeMeters")).doubleValue();
                JSONArray grid = (JSONArray) json.get("grid");
                nodesY = grid.size();
                for (int row = 0; row < grid.size(); row++) {
                    JSONArray rowArray = (JSONArray) grid.get(row);
                    if (row == 0) {
                        nodesX = rowArray.size();
                    }
                    for (int col = 0; col < rowArray.size(); col++) {
                        boolean isObstacle = (boolean) rowArray.get(col);
                        if (isObstacle) {
                            staticObstacles.add(new GridPosition(col, row));
                        }
                    }
                }

                JSONObject fieldSize = (JSONObject) json.get("field_size");
                fieldLength = ((Number) fieldSize.get("x")).doubleValue();
                fieldWidth = ((Number) fieldSize.get("y")).doubleValue();
            } catch (Exception e) {
            }
        }
        // staticObstacles will now contain a list of all the obsticals in the grid
    }

    public boolean checkPosition(Pose2d pose) {
        currentGridPosition = new GridPosition((int) Math.floor(pose.getX() / nodeSize),
                (int) Math.floor(pose.getY() / nodeSize));
        for (GridPosition gridPosition : staticObstacles) {
            if (currentGridPosition.equals(gridPosition)){
                return true;
            }
        }
        return false;
    }
    public ChassisSpeeds CollisionStateSpeeds(SwerveDriveState currentState){
        return CollisionSpeeds(currentState.speeds, currentState.Pose);
    }
    public ChassisSpeeds CollisionSpeeds(ChassisSpeeds speeds, Pose2d pose){
        if (checkPosition(pose)){
            return speeds.unaryMinus().times(1-loss);
        } else {
            return speeds;
        }
    }
}
