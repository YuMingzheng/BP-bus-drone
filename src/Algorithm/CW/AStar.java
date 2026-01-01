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
    // Define a class to represent the state of the search (node in the queue)
    static class AStarNode {
        double[] location;
        List<double[]> path;
        double actualCost;
        double remainingWalkDistance;
        double totalEuclideanDistance;

        public AStarNode(double[] location, List<double[]> path, double actualCost, double remainingWalkDistance, double totalEuclideanDistance) {
            this.location = location;
            this.path = new ArrayList<>(path);
            this.actualCost = actualCost;
            this.remainingWalkDistance = remainingWalkDistance;
            this.totalEuclideanDistance = totalEuclideanDistance;
        }

        // Priority function: f(n) = g(n) + h(n)
        public double getPriority(double[] goal) {
            return this.actualCost + heuristic(this.location, goal);
        }

        // Calculate Euclidean distance between two points (double arrays)
        public static double heuristic(double[] a, double[] b) {
            return Math.sqrt(Math.pow(a[0] - b[0], 2) + Math.pow(a[1] - b[1], 2));
        }
    }

    // A* search function to find the shortest path between two points (start and goal)
    public static Result aStar(List<List<double[]>> routes, double[] start, double[] goal, double R, double walkPenalty) {
        PriorityQueue<AStarNode> queue = new PriorityQueue<>(Comparator.comparingDouble(n -> n.getPriority(goal)));
        Map<String, Double> visited = new HashMap<>();

        queue.add(new AStarNode(start, new ArrayList<>(), 0, R, 0));

        visited.put(Arrays.toString(start), 0.0);

        while (!queue.isEmpty()) {
            AStarNode currentAStarNode = queue.poll();
            double[] current = currentAStarNode.location;
            List<double[]> path = currentAStarNode.path;
            double actualCost = currentAStarNode.actualCost;
            double remainingWalkDistance = currentAStarNode.remainingWalkDistance;
            double totalEuclideanDistance = currentAStarNode.totalEuclideanDistance;

            path.add(current);

            // If we have reached the goal, return the path, the cost, and the total Euclidean distance
            if (Arrays.equals(current, goal)) {
                return new Result(path, actualCost, totalEuclideanDistance);
            }

            // Explore neighbors (i.e., next stops in the same route or interchange stops)
            for (List<double[]> route : routes) {
                if (contains(route, current)) {
                    int currentIndex = indexOf(route, current);

                    // Explore the next stop in the route (forward direction)
                    if (currentIndex + 1 < route.size()) {
                        double[] neighbor = route.get(currentIndex + 1);
                        double euclideanDistance = AStarNode.heuristic(current, neighbor);
                        double newCost = actualCost + euclideanDistance;

                        if (!visited.containsKey(Arrays.toString(neighbor)) || newCost < visited.get(Arrays.toString(neighbor))) {
                            visited.put(Arrays.toString(neighbor), newCost);
                            queue.add(new AStarNode(neighbor, path, newCost, R, totalEuclideanDistance + euclideanDistance));
                        }
                    }

                    // Explore the previous stop in the route (backward direction)
                    if (currentIndex - 1 >= 0) {
                        double[] neighbor = route.get(currentIndex - 1);
                        double euclideanDistance = AStarNode.heuristic(current, neighbor);
                        double newCost = actualCost + euclideanDistance;

                        if (!visited.containsKey(Arrays.toString(neighbor)) || newCost < visited.get(Arrays.toString(neighbor))) {
                            visited.put(Arrays.toString(neighbor), newCost);
                            queue.add(new AStarNode(neighbor, path, newCost, R, totalEuclideanDistance + euclideanDistance));
                        }
                    }
                }
            }

            // Explore interchange possibilities (i.e., switching to other routes)
            if (remainingWalkDistance > 0) {
                for (List<double[]> route : routes) {
                    if (contains(route, current)) continue;

                    for (double[] neighbor : route) {
                        if (!Arrays.equals(neighbor, current)) {
                            double interchangeDistance = AStarNode.heuristic(current, neighbor);
                            if (interchangeDistance <= remainingWalkDistance) {
                                double interchangeCost = interchangeDistance * walkPenalty;
                                double newCost = actualCost + interchangeCost;

                                if (!visited.containsKey(Arrays.toString(neighbor)) || newCost < visited.get(Arrays.toString(neighbor))) {
                                    visited.put(Arrays.toString(neighbor), newCost);
                                    queue.add(new AStarNode(neighbor, path, newCost, remainingWalkDistance - interchangeDistance, totalEuclideanDistance + interchangeDistance));
                                }
                            }
                        }
                    }
                }
            }

            // If the goal is reachable by walking directly
            if (remainingWalkDistance >= AStarNode.heuristic(current, goal)) {
                double euclideanDistance = AStarNode.heuristic(current, goal);
                double newCost = actualCost + euclideanDistance * walkPenalty;

                if (!visited.containsKey(Arrays.toString(goal)) || newCost < visited.get(Arrays.toString(goal))) {
                    visited.put(Arrays.toString(goal), newCost);
                    queue.add(new AStarNode(goal, path, newCost, 0, totalEuclideanDistance + euclideanDistance));
                }
            }
        }

        return new Result(null, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
    }

    // Utility function to check if an array contains a specific point
    public static boolean contains(List<double[]> route, double[] point) {
        for (double[] stop : route) {
            if (Arrays.equals(stop, point)) {
                return true;
            }
        }
        return false;
    }

    // Utility function to get the index of a point in a list
    public static int indexOf(List<double[]> route, double[] point) {
        for (int i = 0; i < route.size(); i++) {
            if (Arrays.equals(route.get(i), point)) {
                return i;
            }
        }
        return -1;
    }

    // Class to store the result (path, cost, and distance)
    static class Result {
        List<double[]> path;
        double cost;
        double distance;

        public Result(List<double[]> path, double cost, double distance) {
            this.path = path;
            this.cost = cost;
            this.distance = distance;
        }
    }


}


