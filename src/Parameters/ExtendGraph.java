package Parameters;

//import IO.CallModel;
import IO.CallModel;
import IO.ModelInputData;
import Old.Parameters;
import Problem.Node.Node;
import Problem.Node.TimeNode;
import org.json.simple.parser.ParseException;

import javax.script.ScriptException;
import java.io.IOException;
import java.util.*;

/**
 * @author Yu Mingzheng
 * @date 2024/9/19 10:09
 * @description
 */
public class ExtendGraph {
    public ArrayList<Node> allNode;

    public ArrayList<Node> V_P;
    public ArrayList<Node> V_D;
    public int[] q_i;
    public List<List<Double>> t_ln;
    public int[][] connectedBusNetwork;
    public int[][] connectedDrone;
    public double[][] distanceMat;
    public int nodeNum;

    public Set<TimeNode> allNodeExtendSet;
    public List<TimeNode> allNodeExtend;
    public double[][] distanceMatExtend;
    public double[][] timeMatDroneExtend;
    public double[][] distanceMatExtendChange;
    public int[][] c1;
    public int[][] c2;
    public int nodeNumExtend;

    public ArrayList<double[]> timeWindowExtend;

    public static Map<TimeNode ,List<TimeNode>> nw;
    /**
     * cost矩阵，用于每次SPPRC中迭代更新
     */
    public double[][] cost;

    public double R;
    public int orderNum;
    public double velB;
    public double velD;
    public double sTime = 0;
    public double stopTime;
    public int droneNum;
    public double T;

    public double[][] edges;
    public double[][] twoOrder;
    public List<List<double[]>> V_S_l;

    public double average_distance;

    public static Map<Node , List<TimeNode>> sameLocation;

    public ExtendGraph(String path , double useModel , String modelName) throws ScriptException, IOException, ParseException {

        Data data = new Data(path);

        this.R = data.R;
        this.orderNum = data.orderNum;
        this.velB = data.velB;
        this.velD = data.velD;
        this.stopTime = data.stopTime;
        this.droneNum = data.droneNum;
        this.T = data.T;
        this.allNode = new ArrayList<>();
        allNode.add(data.depot);

        this.V_P = new ArrayList<>();
        this.V_D = new ArrayList<>();
        for(double[][] pair : data.a){
            V_P.add(new Node(pair[0][0] , pair[0][1]));
            V_D.add(new Node(pair[1][0] , pair[1][1]));
        }
        allNode.addAll(V_P);
        allNode.addAll(V_D);

        List<List<double[]>> V_S_l_data = data.V_S_l;
        List<List<Node>> V_S_L = new ArrayList<>();
        List<Node> allVS = new ArrayList<>();

        for (List<double[]> line : V_S_l_data) {
            List<Node> nodeLine = new ArrayList<>();
            for (double[] point : line) {
                Node node = new Node(point[0], point[1]);
                nodeLine.add(node);
                allVS.add(node);
            }
            V_S_L.add(nodeLine);
        }

        for (int i = 0; i < data.lineNum / 2; i++) {
            List<Node> reversedLine = new ArrayList<>(V_S_L.get(i));
            Collections.reverse(reversedLine);
            V_S_L.add(reversedLine);
        }

        allVS = new ArrayList<>(new LinkedHashSet<>(allVS)); // 去重
        allNode.addAll(allVS);
        allNode.add(data.depot);

        this.nodeNum = allNode.size();
        if(nodeNum != 2 + data.orderNum * 2 + allVS.size()){
            throw new RuntimeException("节点数量不匹配!");
        }

        this.q_i = new int[nodeNum];
        for (int i = 1; i < 1 + data.orderNum * 2 ; i++) {
            if((i-1) / data.orderNum == 0){
                this.q_i[i] = 1;
            }
            if((i-1) / data.orderNum == 1){
                this.q_i[i] = -1;
            }
        }

        // 公交车路线-->转为index
        List<List<Integer>> V_S_L_indices = new ArrayList<>();
        for(List<Node> line : V_S_L){
            List<Integer> lineIndices = new ArrayList<>();
            for(Node node : line){
                int index = allNode.indexOf(node);
                if(index == -1){
                    throw new RuntimeException("节点在allNode中未找到。");
                }
                lineIndices.add(index);
            }
            V_S_L_indices.add(lineIndices);
        }

        this.t_ln = new ArrayList<>();
        for (int i = 0; i < V_S_L_indices.size(); i++) {
            List<Double> t_line = new ArrayList<>();
            if (i < data.lineNum / 2) {
                t_line.add(0.0);
            } else {
                t_line.add(data.resversRunIntv);
            }
            List<Integer> line = V_S_L_indices.get(i);
            for (int j = 0; j < line.size() - 1; j++) {
                double t_prev = t_line.get(j);
                Node node1 = allNode.get(line.get(j));
                Node node2 = allNode.get(line.get(j + 1));
                double dist = calcDist(node1, node2);
                double t_next = t_prev + dist / data.velB + data.stopTime;
                t_line.add(t_next);
            }
            this.t_ln.add(t_line);
        }

        this.connectedBusNetwork = new int[nodeNum][nodeNum];
        for (List<Integer> line : V_S_L_indices) {
            for (int j = 0; j < line.size() - 1; j++) {
                this.connectedBusNetwork[line.get(j)][line.get(j + 1)] = 1;
            }
        }

        this.connectedDrone = new int[nodeNum][nodeNum];
        this.distanceMat = new double[nodeNum][nodeNum];
        for (int i = 0; i < nodeNum; i++) {
            for (int j = 0; j < nodeNum; j++) {
                Node node1 = this.allNode.get(i);
                Node node2 = this.allNode.get(j);
                this.distanceMat[i][j] = calcDist(node1, node2);
                if(this.distanceMat[i][j] <= data.R  && i != j ){
                    this.connectedDrone[i][j] = 1;
                }
            }
        }
        this.connectedDrone[0][nodeNum - 1] = 0;
        this.connectedDrone[nodeNum - 1][0] = 0;



        //-------------------------------
        // 构建Extend Network
        //-------------------------------
        this.allNodeExtendSet = new LinkedHashSet<>();

        // 添加depot
        TimeNode depotTimeNode = new TimeNode(new double[]{data.depot.getX(), data.depot.getY()}, new double[]{0, data.T});
        allNodeExtendSet.add(depotTimeNode);
        // 添加customer node
        for (int i = 1; i <= data.orderNum * 2; i++) {
            Node node = allNode.get(i);
            double[] timeWindow = data.timeWindow.get(i-1);
            TimeNode timeNode = new TimeNode(new double[]{node.getX(), node.getY()} , timeWindow);
            allNodeExtendSet.add(timeNode);
        }
        // Build bus network 'nw' and collect all TimeNodes
        this.nw = new HashMap<>();
        for(int line = 0; line < data.lineNum ; line++){
            List<Integer> lineIndices = V_S_L_indices.get(line);
            List<Double> t_line =  this.t_ln.get(line);
            for (int j = 0; j < lineIndices.size() - 1; j++) {
                double t1 = t_line.get(j);
                double t2 = t_line.get(j + 1);
                while (t1 <= data.T  && t2 <= data.T){
                    Node node1 = allNode.get(lineIndices.get(j));
                    Node node2 = allNode.get(lineIndices.get(j + 1));
                    TimeNode timeNode1 = new TimeNode(new double[]{node1.getX(), node1.getY()}, new double[]{t1, t1 + data.stopTime});
                    TimeNode timeNode2 = new TimeNode(new double[]{node2.getX(), node2.getY()}, new double[]{t2, t2 + data.stopTime});

                    nw.computeIfAbsent(timeNode1, k -> new ArrayList<>()).add(timeNode2);

                    allNodeExtendSet.add(timeNode1);
                    allNodeExtendSet.add(timeNode2);

                    t1+=data.busIntv;
                    t2+=data.busIntv;
                }
            }
        }
        // 添加depot2
        TimeNode depot2TimeNode = new TimeNode(new double[]{data.depot2.getX(), data.depot2.getY()}, new double[]{0, data.T});

        // Convert to List
        this.allNodeExtend = new ArrayList<>(allNodeExtendSet);
        allNodeExtend.add(depot2TimeNode);

        this.nodeNumExtend = allNodeExtend.size();

        this.distanceMatExtend = new double[nodeNumExtend][nodeNumExtend];
        this.distanceMatExtendChange = new double[nodeNumExtend][nodeNumExtend];
        this.c1 = new int[nodeNumExtend][nodeNumExtend];
        this.c2 = new int[nodeNumExtend][nodeNumExtend];


        int c1PlusC2 = 0;
        // Build c2 matrix
        for (Map.Entry<TimeNode, List<TimeNode>> entry : nw.entrySet()) {
            TimeNode i = entry.getKey();
            int idx_i = allNodeExtend.indexOf(i);
            for (TimeNode j : entry.getValue()) {
                int idx_j = allNodeExtend.indexOf(j);
                this.c2[idx_i][idx_j] = 1;
                c1PlusC2++;
            }
        }

        // Build c1 matrix
        for (int i = 0; i < nodeNumExtend; i++) {
            for (int j = 0; j < nodeNumExtend; j++) {
                TimeNode nodeI = allNodeExtend.get(i);
                TimeNode nodeJ = allNodeExtend.get(j);
                this.distanceMatExtend[i][j] = calcDist(nodeI, nodeJ);
                this.distanceMatExtendChange[i][j] = calcDist(nodeI, nodeJ);

                if (this.distanceMatExtend[i][j] <= data.R &&
                        !Arrays.equals(nodeI.getLocation(), nodeJ.getLocation()) &&
                        nodeI.getTimeWindow()[0] < nodeJ.getTimeWindow()[1] &&
                        this.c2[i][j] != 1
                ) {

                    this.c1[i][j] = 1;
                    c1PlusC2++;
                }
            }
        }

        for (int i = 0; i < nodeNumExtend; i++) {
            distanceMatExtend[i][0] = Parameters.bigM;                // 所有点到0的距离为M
            distanceMatExtend[nodeNumExtend-1][i] = Parameters.bigM;  // 终点-1到所有点的距离为M
            distanceMatExtend[i][i] = Parameters.bigM;                // i到i的距离为M

            distanceMatExtendChange[i][0] = Parameters.bigM;
            distanceMatExtendChange[nodeNumExtend-1][i] = Parameters.bigM;
            distanceMatExtendChange[i][i] = Parameters.bigM;
        }

        distanceMatExtend[0][nodeNumExtend-1] = Parameters.bigM;
        distanceMatExtendChange[0][nodeNumExtend-1] = Parameters.bigM;

        // new cost
        this.cost = new double[nodeNumExtend][nodeNumExtend];

        timeWindowExtend = new ArrayList<>();
        for (TimeNode timeNode : allNodeExtend) {
            timeWindowExtend.add(timeNode.getTimeWindow());
        }

        edges = new double[nodeNumExtend][nodeNumExtend];
        twoOrder = new double[orderNum][orderNum];
        this.V_S_l = V_S_l_data;


        sameLocation = new HashMap<>();
        for(Node node : allNode){
            sameLocation.put(node, same_location_all_node(node));
        }


        int n = distanceMatExtend.length;
        double totalDistance = 0.0;
        int count = 0;

        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                totalDistance += distanceMatExtend[i][j];
                count++;
            }
        }

        average_distance = totalDistance / count;


        if(useModel != -1){
            ModelInputData[] modelInputDatas = new ModelInputData[c1PlusC2];
            int counter = 0;
            for (int i = 0; i < this.nodeNumExtend; i++) {
                for (int j = 0; j < this.nodeNumExtend; j++) {
                    if(this.c1[i][j] == 1 || this.c2[i][j] == 1){
                        ModelInputData modelInputData = getModelInputData(i, j, data);
                        modelInputDatas[counter] = modelInputData;
                        counter ++;
                    }
                }
            }

            boolean[] pred = CallModel.predict(modelInputDatas , useModel , modelName);
            counter = 0;
            for (int i = 0; i < this.nodeNumExtend; i++) {
                for (int j = 0; j < this.nodeNumExtend; j++) {
                    if(this.c1[i][j] == 1 || this.c2[i][j] == 1){
                        if(!pred[counter]){
                            this.c1[i][j] = 0;
                            this.c2[i][j] = 0;
                        }
                        counter++;
                    }
                }
            }
        }
    }

    private ModelInputData getModelInputData(int i, int j, Data data) {
        double edge_distance = distanceMatExtend[i][j] / average_distance;
        double depot_2_i = distanceMatExtend[0][i] / average_distance;
        double depot_2_j = distanceMatExtend[0][i] / average_distance;
        double i_time_l = allNodeExtend.get(i).getTimeWindow()[0] / data.T;
        double i_time_r = allNodeExtend.get(i).getTimeWindow()[1] / data.T;
        double j_time_l = allNodeExtend.get(j).getTimeWindow()[0] / data.T;
        double j_time_r = allNodeExtend.get(j).getTimeWindow()[1] / data.T;

        int edge_type = this.c1[i][j] == 1 ? 1 : 2;
        int has_customer;
        if((1 <= i && i <= this.orderNum*2+1) || ((1 <= j && j <= this.orderNum*2+1))){
             has_customer = 1;
        }else{
            has_customer = 0;
        }

        ModelInputData modelInputData = new ModelInputData();

        modelInputData.setDepot_2_i(depot_2_i);
        modelInputData.setDepot_2_j(depot_2_j);
        modelInputData.setI_time_l(i_time_l);
        modelInputData.setI_time_r(i_time_r);
        modelInputData.setJ_time_l(j_time_l);
        modelInputData.setJ_time_r(j_time_r);
        modelInputData.setEdge_type(edge_type);
        modelInputData.setHas_customer(has_customer);
        modelInputData.setEdge_distance(edge_distance);
        return modelInputData;
    }

    // 实现same_location_all_node方法：查找位置相同的所有节点
    public  List<TimeNode> same_location_all_node(Node location) {
        List<TimeNode> same = new ArrayList<>();
        double[] locationCoords = location.getLocation();
        for (TimeNode node : this.allNodeExtend) {
            if (Arrays.equals(node.getLocation(), locationCoords)) {
                same.add(node);
            }
        }
        return same;
    }

    public static TimeNode closest_time_node(Node location , double arrival_time , Node next_location , boolean is_PD){
        List<TimeNode> locations = sameLocation.get(location);
        locations.sort(Comparator.comparingDouble(o -> o.getTimeWindow()[0]));

        if(is_PD){
            return locations.get(0);
        }

        for (TimeNode loca : locations) {
            if (loca.getTimeWindow()[0] >= arrival_time){
                List<TimeNode> neighbors = nw.getOrDefault(loca , null);
                if(neighbors != null) {
                    for (TimeNode neighbor : neighbors) {
                        if (Arrays.equals(neighbor.getLocation(), next_location.getLocation())) {
                            return loca;
                        }
                    }
                }
            }
        }
        return null;

    }


    public static double calcDist(Node x, Node y) {
        return Math.sqrt(Math.pow(x.getX() - y.getX(), 2) + Math.pow(x.getY() - y.getY(), 2));
    }

}
