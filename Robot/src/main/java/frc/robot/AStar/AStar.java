package frc.robot.AStar;

// import java.io.*;
// import java.lang.*;
// import java.util.*;

// Dijkstra's Algorithm in Java

public class AStar {

  public static void algorithm(int[][] graph, int source) {
    int count = graph.length;
    boolean[] visitedVertex = new boolean[count]; //initializes all values to false
    int[] distance = new int[count]; //distances from source to... 
    for (int i = 0; i < count; i++) {
      visitedVertex[i] = false;
      distance[i] = Integer.MAX_VALUE;
    }

    // Distance of self loop is zero
    distance[source] = 0;
    for (int i = 0; i < count; i++) {

      // Update the distance between neighbouring vertex and source vertex
      int u = findMinDistance(distance, visitedVertex); //pass in array of distances and vertexes
      visitedVertex[u] = true;

      // Update all the neighbouring vertex distances
      for (int v = 0; v < count; v++) {
        if (!visitedVertex[v] && graph[u][v] != 0 && (distance[u] + graph[u][v] < distance[v])) {
          //if... we haven't visited the vertex, and the graph at that value isn't zeor, and the new distance is less than the current distance.
          distance[v] = distance[u] + graph[u][v];
        }
      }
    }
    for (int i = 0; i < distance.length; i++) {
      System.out.println(String.format("Distance from %s to %s is %s", source, i, distance[i]));
    }

  }

  private static int findMinDistance(int[] distance, boolean[] visitedVertex) {
    int minDistance = Integer.MAX_VALUE;
    int minDistanceVertex = -1;
    for (int i = 0; i < distance.length; i++) {
      if (!visitedVertex[i] && distance[i] < minDistance) {
        minDistance = distance[i];
        minDistanceVertex = i;
      }
    }
    return minDistanceVertex;
  }

  public static void main(String[] args) {
    int graph[][] = new int[][] {
        { 0, 0, 1, 2, 0, 0, 0 },
        { 0, 0, 2, 0, 0, 3, 0 },
        { 1, 2, 0, 1, 3, 0, 0 },
        { 2, 0, 1, 0, 0, 0, 1 },
        { 0, 0, 3, 0, 0, 2, 0 },
        { 0, 3, 0, 0, 2, 0, 1 },
        { 0, 0, 0, 1, 0, 1, 0 } };
    algorithm(graph, 0); // starts the algorithm with the given graph
  }
}