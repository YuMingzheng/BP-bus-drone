import Parameters.Data;
import org.json.simple.parser.ParseException;

import javax.script.ScriptException;
import java.io.IOException;
import java.util.*;
import Problem.Node.Node;
import Problem.Node.TimeNode;

/**
 * @author Yu Mingzheng
 * @date 2024/9/13 10:02
 * @description
 */
class Main2 {
    public static void main(String[] args) {
        try {
            Data data = new Data("C:\\Users\\31706\\Desktop\\bus+drone\\bus-drone-code\\data\\data_file\\intance1.json");

            // 构建网络
            double T = data.T;
            int droneNum = data.droneNum;
            int orderNum = data.orderNum;
            int lineNum = data.lineNum; // 往返，乘2
            double busIntv = data.busIntv;
            double stopTime = data.stopTime;
            double velB = data.velB;
            double velD = data.velD;
            double R = data.R;
            double resversRunIntv = data.resversRunIntv;

            Node depot = data.depot;
            Node depot2 = data.depot2;
            List<Node> allNode = new ArrayList<>();
            allNode.add(depot);

            double[][][] a = data.a;
            List<double[]> timeWindowData = data.timeWindow;

            List<Node> V_P = new ArrayList<>();
            List<Node> V_D = new ArrayList<>();
            for (double[][] pair : a) {
                V_P.add(new Node(pair[0][0], pair[0][1]));
                V_D.add(new Node(pair[1][0], pair[1][1]));
            }
            allNode.addAll(V_P);
            allNode.addAll(V_D);

            List<List<double[]>> V_S_l_data = data.V_S_l;
            List<List<Node>> V_S_l = new ArrayList<>();
            List<Node> allVS = new ArrayList<>();

            for (List<double[]> line : V_S_l_data) {
                List<Node> nodeLine = new ArrayList<>();
                for (double[] point : line) {
                    Node node = new Node(point[0], point[1]);
                    nodeLine.add(node);
                    allVS.add(node);
                }
                V_S_l.add(nodeLine);
            }

            int originalLineNum = V_S_l.size();
            for (int i = 0; i < originalLineNum; i++) {
                List<Node> reversedLine = new ArrayList<>(V_S_l.get(i));
                Collections.reverse(reversedLine);
                V_S_l.add(reversedLine);
            }
            lineNum = V_S_l.size();

            Set<Node> allVSSet = new HashSet<>(allVS);
            allVS = new ArrayList<>(allVSSet);
            allNode.addAll(allVS);
            allNode.add(depot2);

            int nodeNum = allNode.size();
            if (nodeNum != 2 + orderNum * 2 + allVS.size()) {
                throw new RuntimeException("节点数量不匹配。");
            }

            int[] q_i = new int[nodeNum];
            for (int i = 1; i < 1 + orderNum * 2; i++) {
                if ((i - 1) / orderNum == 0) {
                    q_i[i] = 1;
                }
                if ((i - 1) / orderNum == 1) {
                    q_i[i] = -1;
                }
            }

            // 公交车路线-->转换为index
            List<List<Integer>> V_S_l_indices = new ArrayList<>();
            for (List<Node> line : V_S_l) {
                List<Integer> lineIndices = new ArrayList<>();
                for (Node node : line) {
                    int index = allNode.indexOf(node);
                    if (index == -1) {
                        throw new RuntimeException("节点在allNode中未找到。");
                    }
                    lineIndices.add(index);
                }
                V_S_l_indices.add(lineIndices);
            }

            List<List<Double>> t_ln = new ArrayList<>();
            for (int i = 0; i < V_S_l_indices.size(); i++) {
                List<Double> t_line = new ArrayList<>();
                if (i < lineNum / 2) {
                    t_line.add(0.0);
                } else {
                    t_line.add(resversRunIntv);
                }
                List<Integer> line = V_S_l_indices.get(i);
                for (int j = 0; j < line.size() - 1; j++) {
                    double t_prev = t_line.get(j);
                    Node node1 = allNode.get(line.get(j));
                    Node node2 = allNode.get(line.get(j + 1));
                    double dist = calcDist(node1, node2);
                    double t_next = t_prev + dist / velB + stopTime;
                    t_line.add(t_next);
                }
                t_ln.add(t_line);
            }

            int[][] connectedBusNetwork = new int[nodeNum][nodeNum];
            for (List<Integer> line : V_S_l_indices) {
                for (int j = 0; j < line.size() - 1; j++) {
                    connectedBusNetwork[line.get(j)][line.get(j + 1)] = 1;
                }
            }

            int[][] connectedDrone = new int[nodeNum][nodeNum];
            double[][] distanceMat = new double[nodeNum][nodeNum];
            for (int i = 0; i < nodeNum; i++) {
                for (int j = 0; j < nodeNum; j++) {
                    Node nodeI = allNode.get(i);
                    Node nodeJ = allNode.get(j);
                    distanceMat[i][j] = calcDist(nodeI, nodeJ);
                    if (distanceMat[i][j] <= R && i != j) {
                        connectedDrone[i][j] = 1;
                    }
                }
            }
            connectedDrone[0][nodeNum - 1] = 0;
            connectedDrone[nodeNum - 1][0] = 0;




            // 构建Extend network
            Set<TimeNode> allNodeExtendSet = new LinkedHashSet<>();

            // 添加 depot node
            TimeNode depotTimeNode = new TimeNode(new double[]{depot.getX(), depot.getY()}, new double[]{0, T});
            allNodeExtendSet.add(depotTimeNode);

            // 添加 customer nodes
            for (int i = 1; i <= orderNum * 2; i++) {
                Node node = allNode.get(i);
                double[] timeWindow = data.timeWindow.get(i - 1);
                TimeNode timeNode = new TimeNode(new double[]{node.getX(), node.getY()}, timeWindow);
                allNodeExtendSet.add(timeNode);
            }

            // Build bus network 'nw' and collect all TimeNodes
            Map<TimeNode, List<TimeNode>> nw = new HashMap<>();
            for (int line = 0; line < lineNum; line++) {
                List<Integer> lineIndices = V_S_l_indices.get(line);
                List<Double> t_line = t_ln.get(line);
                for (int j = 0; j < lineIndices.size() - 1; j++) {
                    double t1 = t_line.get(j);
                    double t2 = t_line.get(j + 1);
                    while (t1 <= T && t2 <= T) {
                        Node node1 = allNode.get(lineIndices.get(j));
                        Node node2 = allNode.get(lineIndices.get(j + 1));
                        TimeNode timeNode1 = new TimeNode(new double[]{node1.getX(), node1.getY()}, new double[]{t1, t1 + stopTime});
                        TimeNode timeNode2 = new TimeNode(new double[]{node2.getX(), node2.getY()}, new double[]{t2, t2 + stopTime});

                        nw.computeIfAbsent(timeNode1, k -> new ArrayList<>()).add(timeNode2);

                        // Add TimeNodes to allNodeExtendSet
                        allNodeExtendSet.add(timeNode1);
                        allNodeExtendSet.add(timeNode2);

                        t1 += busIntv;
                        t2 += busIntv;
                    }
                }
            }

            // 添加 depot2 node
            TimeNode depot2TimeNode = new TimeNode(new double[]{depot2.getX(), depot2.getY()}, new double[]{0, T});
//            allNodeExtendSet.add(depot2TimeNode);

            // Convert to List
            List<TimeNode> allNodeExtend = new ArrayList<>(allNodeExtendSet);
            allNodeExtend.add(depot2TimeNode);

            int nodeNumExtend = allNodeExtend.size();

            double[][] distanceMatExtend = new double[nodeNumExtend][nodeNumExtend];

            int[][] c1 = new int[nodeNumExtend][nodeNumExtend];
            int[][] c2 = new int[nodeNumExtend][nodeNumExtend];

            // Build c2 matrix
            for (Map.Entry<TimeNode, List<TimeNode>> entry : nw.entrySet()) {
                TimeNode i = entry.getKey();
                int idx_i = allNodeExtend.indexOf(i);
                for (TimeNode j : entry.getValue()) {
                    int idx_j = allNodeExtend.indexOf(j);
                    c2[idx_i][idx_j] = 1;
                }
            }

            // Build c1 matrix
            for (int i = 0; i < nodeNumExtend; i++) {
                for (int j = 0; j < nodeNumExtend; j++) {
                    TimeNode nodeI = allNodeExtend.get(i);
                    TimeNode nodeJ = allNodeExtend.get(j);
                    distanceMatExtend[i][j] = calcDist(nodeI, nodeJ);

                    if (distanceMatExtend[i][j] <= R &&
                            !Arrays.equals(nodeI.getLocation(), nodeJ.getLocation()) &&
                            nodeI.getTimeWindow()[0] < nodeJ.getTimeWindow()[1] &&
                            c2[i][j] != 1) {
                        c1[i][j] = 1;
                    }
                }
            }

        } catch (IOException | ParseException | ScriptException e) {
            e.printStackTrace();
        }
    }

    public static double calcDist(Node x, Node y) {
        return Math.sqrt(Math.pow(x.getX() - y.getX(), 2) + Math.pow(x.getY() - y.getY(), 2));
    }
}

