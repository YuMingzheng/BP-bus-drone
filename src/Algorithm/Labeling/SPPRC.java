package Algorithm.Labeling;

import Parameters.Parameters;
import Problem.Route;

import java.util.*;

/**
 * @author Yu Mingzheng
 * @date 2024/9/3 15:41
 * @description
 */
public class SPPRC{
    public int vertexNum;
    public Parameters parameters;

    /**待处理的label*/
    public PriorityQueue<Label> unprocessedLabels;

    /**每个node id对应的labels*/
    public ArrayList<ArrayList<Label>> labelList;

    public ArrayList<Route> shortestPaths;
    public ArrayList<Label> optLabels;

    public SPPRC(Parameters parameters){
        this.parameters = parameters;

        vertexNum = Parameters.numClient +2;
        shortestPaths = new ArrayList<>();

        unprocessedLabels = new PriorityQueue<>(new LabelComparator());

        labelList = new ArrayList<>();
        for (int i = 0; i < vertexNum; i++) {
            labelList.add(new ArrayList<>());
        }
    }

    /**
     * Step 0: 初始化
     * Step 1: 选择一个label进行拓展
     * Step 2: Extension
     * Step 3: Dominance
     * Step 4: Filtering
     * @param lambda
     */
    public void solve(Map<Integer , Double> lambda , ArrayList<Route> bestRoutesReturn){
        this.reset();
        this.updateCostMatrix(lambda);

        // 【Step 0】初始化
        Label initialLabel = new Label(0,0,0,0);
        labelList.get(0).add(initialLabel);
        unprocessedLabels.offer(initialLabel);

        while(!unprocessedLabels.isEmpty()){
            //【Step 1】选择一个要进行extend的label
            Label currLabel = unprocessedLabels.poll();

            //【Step 2 & 3】进行extend，判断是否dominated
            for (int i = 0; i < vertexNum; i++) {
                if(parameters.time[currLabel.vertexId][i] == Parameters.bigM) {
                    continue;
                }
                this.labelExtension(currLabel , i);
            }
        }

        this.optLabels = this.filtering(labelList.get(vertexNum-1));


        for (Label optLabel : optLabels) {
            Route newRoute = new Route();
            newRoute.setCost(optLabel.cost);
            ArrayList<Integer> visitVertex = optLabel.getVisitVertexes();
            for (Integer vertex : visitVertex) {
                newRoute.addCity(vertex);
            }
            bestRoutesReturn.add(newRoute);
        }
    }

    public void labelExtension(Label currLabel , int nextVertexId){
        // 拓展的node不能是当前label的node
        if(nextVertexId == currLabel.vertexId || currLabel.visitedNode(nextVertexId))
            return ;
        // check当前的extend在载货量、time上是否可行
        double demand = currLabel.demand + parameters.demand[nextVertexId];
        if(demand > parameters.vehCapacity)
            return ;
        double time = currLabel.time + parameters.s[currLabel.vertexId] + parameters.time[currLabel.vertexId][nextVertexId];
        if(time > parameters.b[nextVertexId])
            return;
        if(time < parameters.a[nextVertexId])
            time = parameters.a[nextVertexId];
        double cost = currLabel.cost + parameters.cost[currLabel.vertexId][nextVertexId];


        Label newLabel = new Label(cost , time,  demand , nextVertexId , currLabel);
        // 【步骤3】check一下new label是否被dominated
        this.useDominanceRules(newLabel);
    }

//    public void useDominanceRules(Label labelToCompare) {
//        // 获取new label 的id，只跟id相同的label比较
//        int currVertexId = labelToCompare.vertexId;
//        ArrayList<Label> processedLabels = labelList.get(currVertexId);
//
//        boolean isDominated = false;
//        boolean isPossibleDominatedByNextLabel = true;
//
//        Iterator<Label> iterator = processedLabels.iterator();
//        while (iterator.hasNext()){
//            Label other = iterator.next();
//
//            if(labelToCompare.dominate(other)){
//                if(isPossibleDominatedByNextLabel && labelToCompare.equals(other))
//                    return;
//
//                isPossibleDominatedByNextLabel = false;
//                unprocessedLabels.remove(other);
//                iterator.remove();
//            }
//            if(isPossibleDominatedByNextLabel && other.dominate(labelToCompare)){
//                isDominated = true;
//                break;
//            }
//        }
//        if(!isDominated){
//            processedLabels.add(labelToCompare);
//            if(currVertexId != vertexNum - 1)
//                unprocessedLabels.offer(labelToCompare);
//        }
//    }

    public void useDominanceRules(Label labelToCompare) {
        // 获取new label 的id，只跟id相同的label比较
        int currVertexId = labelToCompare.vertexId;
        ArrayList<Label> processedLabels = labelList.get(currVertexId);

        boolean isDominated = false;

        Iterator<Label> iterator = processedLabels.iterator();
        while (iterator.hasNext()){
            Label other = iterator.next();
            if(labelToCompare.dominate(other)){
                if(labelToCompare.equals(other))
                    return;
                unprocessedLabels.remove(other);
                iterator.remove();
            }
            if(other.dominate(labelToCompare)){
                isDominated = true;
                break;
            }
        }
        if(!isDominated){
            processedLabels.add(labelToCompare);
            if(currVertexId != vertexNum - 1)
                unprocessedLabels.offer(labelToCompare);
        }
    }


    protected void reset(){
        this.unprocessedLabels.clear();
        this.labelList.forEach(ArrayList::clear);
        this.shortestPaths.clear();
    }

    public void updateCostMatrix(Map<Integer , Double> newDualValues){
        for (int i = 0; i < vertexNum; i++) {
            for (int j = 0; j < vertexNum; j++) {
                parameters.cost[i][j] = parameters.distBase[i][j];
            }
        }

        for(Map.Entry<Integer ,Double> entry : newDualValues.entrySet()){
            for (int j = 0; j < vertexNum; j++) {
                parameters.cost[entry.getKey()][j] = parameters.cost[entry.getKey()][j] - entry.getValue();
            }
        }
    }

    private static class LabelComparator implements Comparator<Label> {
        public int compare(Label first , Label second){
            if (first.cost < second.cost)
                return -1;
            if (first.cost > second.cost)
                return 1;
            if (first.time < second.time)
                return -1;
            if (first.time > second.time)
                return 1;
            if (first.demand < second.demand)
                return -1;
            if (first.demand > second.demand)
                return 1;
            return 0;
        }
    }

    public ArrayList<Label> filtering(ArrayList<Label> allFinalLabels) {
        if (allFinalLabels.isEmpty()) {
            throw new NullPointerException("未找到最短路径");
        }
        allFinalLabels.removeIf(label -> label.cost > - Parameters.EPS);
        return allFinalLabels;
    }
    public static void main(String[] args) {
        double start = System.currentTimeMillis();
        Parameters parameters = new Parameters("C:\\Users\\31706\\Desktop\\exact-algorithm\\instances\\solomon_100\\c109.txt");



        ArrayList<Route> bestRoutes = new ArrayList<>();
        Map<Integer, Double> dualPrices = new HashMap<>(Parameters.numClient);
        for (int i = 1; i <= Parameters.numClient; i++)
            dualPrices.put(i, 7d);


        SPPRC sp = new SPPRC(parameters);
        sp.solve(dualPrices , bestRoutes);



//        int vertexNum = parameters.numClients+2;
//        for (int i = 0; i < vertexNum; i++)
//            for (int j = 0; j < vertexNum; j++)
//                parameters.cost[i][j] = parameters.distBase[i][j];
//
//        for(Map.Entry<Integer ,Double> entry : dualPrices.entrySet())
//            for (int j = 0; j < vertexNum; j++)
//                parameters.cost[entry.getKey()][j] = parameters.cost[entry.getKey()][j] - entry.getValue();
//
//        SPPRC spprc = new SPPRC();
//        spprc.shortestPath(parameters , bestRoutes , parameters.numClients);


        System.out.println("Time consumption: " + (System.currentTimeMillis() - start) / 1000.0);

        System.out.println("Path: " + bestRoutes);
        System.out.println(bestRoutes.size());

    }
}
