package Algorithm.CW;
import Parameters.Data;
import Parameters.ExtendGraph;
import org.json.simple.parser.ParseException;

import javax.script.ScriptException;
import java.io.IOException;
import java.util.*;

/**
 * @author Yu Mingzheng
 * @date 2024/10/30 14:35
 * @description
 */

public class AStar {
    // Define the heuristic function to estimate distance between two points using Euclidean distance
    private static double heuristic(double[] a, double[] b) {
        return Math.sqrt(Math.pow(a[0] - b[0], 2) + Math.pow(a[1] - b[1], 2));
    }

    // A* search function to find the shortest path between two bus stops or arbitrary locations
    public static PathResult aStar(List<List<double[]>> routes, double[] start, double[] goal, double R, double walkPenalty) {
        // Create a priority queue to store nodes for exploration
        PriorityQueue<Node> queue = new PriorityQueue<>(Comparator.comparingDouble(n -> n.estimatedCost));
        queue.add(new Node(0, start, new ArrayList<>(), 0, R, 0));

        // Dictionary to store the cost of reaching each node
        Map<String, Double> visited = new HashMap<>();
        visited.put(Arrays.toString(start), 0.0);

        while (!queue.isEmpty()) {
            Node currentNode = queue.poll();
            double[] current = currentNode.currentNode;
            List<double[]> path = new ArrayList<>(currentNode.path);
            path.add(current);

            // If we have reached the goal, return the path, the cost, and the total Euclidean distance
            if (Arrays.equals(current, goal)) {
                return new PathResult(path, currentNode.actualCost, currentNode.totalEuclideanDistance);
            }

            // Explore neighbors (i.e., next stops in the same route or interchange stops)
            for (List<double[]> route : routes) {
                if (Arrays.asList(route).contains(current)) {
                    int currentIndex = Arrays.asList(route).indexOf(current);
                    // Explore the next stop in the route (forward direction)
                    if (currentIndex + 1 < route.size()) {
                        double[] neighbor = route.get(currentIndex + 1);
                        processNeighbor(queue, current, neighbor, currentNode, goal, visited, walkPenalty);
                    }
                    // Explore the previous stop in the route (backward direction)
                    if (currentIndex - 1 >= 0) {
                        double[] neighbor = route.get(currentIndex - 1);
                        processNeighbor(queue, current, neighbor, currentNode, goal, visited, walkPenalty);
                    }
                }

                // Explore interchange possibilities (i.e., switching to other routes)
                if (currentNode.remainingWalkDistance > 0) {
                    for (List<double[]> otherRoute : routes) {
                        if (Arrays.asList(otherRoute).contains(current)) {
                            continue;
                        }
                        for (double[] neighbor : otherRoute) {
                            if (!Arrays.equals(neighbor, current)) {
                                double interchangeDistance = heuristic(current, neighbor);
                                if (interchangeDistance <= currentNode.remainingWalkDistance) {
                                    double interchangeCost = interchangeDistance * walkPenalty;
                                    double newCost = currentNode.actualCost + interchangeCost;
                                    processNeighbor(queue, current, neighbor, new Node(newCost, neighbor, path, newCost, currentNode.remainingWalkDistance - interchangeDistance, currentNode.totalEuclideanDistance + interchangeDistance), goal, visited, walkPenalty);
                                }
                            }
                        }
                    }
                }
            }

            // If goal is not directly reachable via bus routes, explore straight-line walking to the goal
            if (currentNode.remainingWalkDistance >= heuristic(current, goal)) {
                double euclideanDistance = heuristic(current, goal);
                double newCost = currentNode.actualCost + euclideanDistance * walkPenalty;
                if (!visited.containsKey(Arrays.toString(goal)) || newCost < visited.get(Arrays.toString(goal))) {
                    visited.put(Arrays.toString(goal), newCost);
                    queue.add(new Node(newCost, goal, path, newCost, 0, currentNode.totalEuclideanDistance + euclideanDistance));
                }
            }
        }

        // If the goal cannot be reached, return null
        return new PathResult(null, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
    }

    // Helper method to process neighbor nodes
    private static void processNeighbor(PriorityQueue<Node> queue, double[] current, double[] neighbor, Node currentNode, double[] goal, Map<String, Double> visited, double walkPenalty) {
        double euclideanDistance = heuristic(current, neighbor);
        double newCost = currentNode.actualCost + euclideanDistance;
        if (!visited.containsKey(Arrays.toString(neighbor)) || newCost < visited.get(Arrays.toString(neighbor))) {
            visited.put(Arrays.toString(neighbor), newCost);
            double priority = newCost + heuristic(neighbor, goal);
            queue.add(new Node(priority, neighbor, currentNode.path, newCost, currentNode.remainingWalkDistance, currentNode.totalEuclideanDistance + euclideanDistance));
        }
    }

    // Node class to hold state for A* search
    static class Node {
        double estimatedCost;
        double[] currentNode;
        List<double[]> path;
        double actualCost;
        double remainingWalkDistance;
        double totalEuclideanDistance;

        Node(double estimatedCost, double[] currentNode, List<double[]> path, double actualCost, double remainingWalkDistance, double totalEuclideanDistance) {
            this.estimatedCost = estimatedCost;
            this.currentNode = currentNode;
            this.path = path;
            this.actualCost = actualCost;
            this.remainingWalkDistance = remainingWalkDistance;
            this.totalEuclideanDistance = totalEuclideanDistance;
        }
    }

    // Result class to return path results
    static class PathResult {
        List<double[]> path;
        double cost;
        double distance;

        PathResult(List<double[]> path, double cost, double distance) {
            this.path = path;
            this.cost = cost;
            this.distance = distance;
        }
    }

    // Example usage
    public static void main(String[] args) throws ScriptException, IOException, ParseException {
        Data data = new Data("C:\\Users\\31706\\Desktop\\bus+drone\\bus-drone-code\\data\\write_json.json");
        ExtendGraph extendGraph = new ExtendGraph();
        // Define your routes and start/goal locations here for testing

        // Populate routes with bus stop coordinates as needed

        double[] start = extendGraph.allNode.get(0).getLocation();
        double[] goal = extendGraph.allNode.get(1).getLocation();
        double R = 10;            // Maximum walking distance
        double walkPenalty = 10;  // Walking penalty

        PathResult result = aStar(data.V_S_l, start, goal, R, walkPenalty);

        if (result.path != null) {
            System.out.println("Path: " + result.path);
            System.out.println("Cost: " + result.cost);
            System.out.println("Distance: " + result.distance);
        } else {
            System.out.println("Goal cannot be reached.");
        }
    }
}


