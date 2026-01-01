package Algorithm.Labeling;

import Old.Parameters;
import Parameters.ExtendGraph;
import Problem.Route;
import org.json.simple.parser.ParseException;

import javax.script.ScriptException;
import java.io.IOException;
import java.util.*;

/**
 * @author Yu Mingzheng
 * @date 2024/9/9 09:45
 * @description
 */
public class MySPPRC {
    public int vertexNum;
    public ExtendGraph extendGraph;

    /**待处理的label*/
    public PriorityQueue<MyLabel> unprocessedLabels;

    /**每个node id对应的labels*/
    public ArrayList<ArrayList<MyLabel>> labelList;

    public ArrayList<Route> shortestPaths;
    public ArrayList<MyLabel> optLabels;

    public MySPPRC(ExtendGraph extendGraph) {

        this.extendGraph = extendGraph;

        vertexNum = extendGraph.nodeNumExtend;
        shortestPaths = new ArrayList<>();

        unprocessedLabels = new PriorityQueue<>(new MyLabelComparator());

        labelList = new ArrayList<>();
        for (int i = 0; i < vertexNum; i++) {
            labelList.add(new ArrayList<>());
        }

    }

    /**
     * 求解步骤：
     * Step 0: 初始化
     * Step 1: 选择一个Label进行拓展
     * Step 2: Extension
     * Step 3: Dominance
     * Step 4: Filtering
     * @param lambda 对偶值
     * @param bestRouteReturn 返回的最优路径
     */
    public void solve(Map<Integer , Double> lambda , ArrayList<Route> bestRouteReturn,double piSingle){
        this.reset();
        this.updateCostMatrix(lambda);

        /*
          【Step 0】初始化
         */
        MyLabel initialLabel = new MyLabel(0,0,0,extendGraph.R,0,0,new boolean[extendGraph.orderNum] , new boolean[extendGraph.orderNum]);
        labelList.get(0).add(initialLabel);
        unprocessedLabels.add(initialLabel);

        while(!unprocessedLabels.isEmpty()){
            /*
            【Step 1】选择一个要进行extend的label
             */
            MyLabel currLabel = unprocessedLabels.poll();

            /*
            【Step 2 & 3】进行extend，判断是否dominated
             */
            for (int i = 0; i < vertexNum; i++) {
                if(extendGraph.distanceMatExtend[currLabel.vertexId][i] == Parameters.bigM) {
                    continue;
                }
                this.labelExtension(currLabel , i);
            }
        }

        this.optLabels = this.filtering(labelList.get(vertexNum-1));

//        System.out.println("\n -- opt label size : " + optLabels.size());

        this.optLabels.sort(Comparator.comparingDouble(label -> label.cost));


        for (MyLabel optLabel : optLabels) {

            Route newRoute = new Route();
            newRoute.setDistance(optLabel.cost - piSingle);
            ArrayList<Integer> visitVertex = optLabel.getVisitVertexes();
            for (Integer vertex : visitVertex) {
                newRoute.addCity(vertex);
            }
            for (int i = 0; i < optLabel.servedReq.length; i++) {
                if(optLabel.servedReq[i]){
                    newRoute.throughOrder.add(i+1);
                }
            }
            bestRouteReturn.add(newRoute);

        }


    }

    /**
     * 1、更新isLoad，判断是否可行
     * 2、更新续航duration，判断是否可行
     * 3、更新arrivalTime，判断是否可行
     * 4、更新cost、serviceTime、openReq、servedReq
     * @param currLabel 当前label
     * @param nextVertexId 下一个id
     */
    public void labelExtension(MyLabel currLabel , int nextVertexId) {
        if(nextVertexId == currLabel.vertexId || currLabel.visitedNode(nextVertexId))
            return;

        int isLoad;
        double cost , duration , arrivalTime , serviceTime ;
        boolean[] openReq , servedReq;
        MyLabel newLabel = null;


        cost = currLabel.cost + extendGraph.cost[currLabel.vertexId][nextVertexId];

        // 判断curr->next是否距离可行
        if(extendGraph.distanceMatExtendChange[currLabel.vertexId][nextVertexId] > Parameters.bigM - Parameters.EPS){
            return ;
        }


        /*
        Case 1 : j in V_P
         */
        if(1 <= nextVertexId && nextVertexId <= extendGraph.orderNum){
            /*
            Subcase 1.1 : i in V_S+{0}
             */
            if(currLabel.vertexId == 0 || (1 + extendGraph.orderNum * 2 <= currLabel.vertexId && currLabel.vertexId <= extendGraph.nodeNumExtend - 2)){
                isLoad = currLabel.isLoad + 1;
                if(isLoad > 1) //TODO 无人机容量这里只设置为1
                    return ;

                duration = currLabel.duration - extendGraph.distanceMatExtend[currLabel.vertexId][nextVertexId];
                if(duration < 0)
                    return ;

                arrivalTime = currLabel.serviceTime + extendGraph.distanceMatExtend[currLabel.vertexId][nextVertexId] / extendGraph.velD;
                if(arrivalTime > extendGraph.timeWindowExtend.get(nextVertexId)[1])
                    return;

                serviceTime = Math.max(arrivalTime , extendGraph.timeWindowExtend.get(nextVertexId)[0]);
                openReq = currLabel.openReq.clone();
                openReq[nextVertexId-1] = true;

                servedReq = currLabel.servedReq.clone();
                newLabel = new MyLabel(nextVertexId, cost, isLoad, duration, arrivalTime, serviceTime, openReq, servedReq , currLabel);
            }
            /*
            Subcase 1.2 : i in V_D
             */
            else if(1 + extendGraph.orderNum <= currLabel.vertexId && currLabel.vertexId <= extendGraph.orderNum * 2){
                isLoad = currLabel.isLoad + 1;
                if(isLoad > 1) //TODO 无人机容量这里只设置为1
                    return ;

                duration = currLabel.duration - extendGraph.distanceMatExtend[currLabel.vertexId][nextVertexId];
                if(duration < 0)
                    return ;

                arrivalTime = currLabel.serviceTime + extendGraph.distanceMatExtend[currLabel.vertexId][nextVertexId] / extendGraph.velD + extendGraph.sTime;
                if(arrivalTime > extendGraph.timeWindowExtend.get(nextVertexId)[1])
                    return;

                serviceTime = Math.max(arrivalTime , extendGraph.timeWindowExtend.get(nextVertexId)[0]);
                openReq = currLabel.openReq.clone();
                openReq[nextVertexId-1] = true;

                servedReq = currLabel.servedReq.clone();
                newLabel = new MyLabel(nextVertexId, cost, isLoad, duration, arrivalTime, serviceTime, openReq, servedReq , currLabel);
            }
            /*
            Subcase 1.3 : i in V_P
             */
            else if(currLabel.vertexId <= extendGraph.orderNum){
                // 什么也不做
            }
            else{
                System.out.println("!!!!!!!!!error");
            }
        }

        /*
        Case 2 : j in V_D
         */
        else if(1 + extendGraph.orderNum <= nextVertexId && nextVertexId <= extendGraph.orderNum * 2){
            if(currLabel.openReq[nextVertexId-extendGraph.orderNum-1]) {
                /*
                Subcase 2.1 : i in V_S+{0}
                 */
                if (currLabel.vertexId == 0 || (1 + extendGraph.orderNum * 2 <= currLabel.vertexId && currLabel.vertexId <= extendGraph.nodeNumExtend - 2)) {
                    isLoad = currLabel.isLoad - 1;
                    if (isLoad < 0)
                        return;

                    duration = currLabel.duration - extendGraph.distanceMatExtend[currLabel.vertexId][nextVertexId];
                    if (duration < 0)
                        return;

                    arrivalTime = currLabel.serviceTime + extendGraph.distanceMatExtend[currLabel.vertexId][nextVertexId] / extendGraph.velD;
                    if (arrivalTime > extendGraph.timeWindowExtend.get(nextVertexId)[1])
                        return;


                    serviceTime = Math.max(arrivalTime, extendGraph.timeWindowExtend.get(nextVertexId)[0]);
                    openReq = currLabel.openReq.clone();
                    openReq[nextVertexId - extendGraph.orderNum - 1] = false;
                    servedReq = currLabel.servedReq.clone();
                    servedReq[nextVertexId - extendGraph.orderNum - 1] = true;

                    newLabel = new MyLabel(nextVertexId, cost, isLoad, duration, arrivalTime, serviceTime, openReq, servedReq, currLabel);
                }
                /*
                Subcase 2.2 : i in V_P
                 */
                else if (currLabel.vertexId <= extendGraph.orderNum) {
                    if (currLabel.vertexId == nextVertexId - extendGraph.orderNum) {
                        isLoad = currLabel.isLoad - 1;
                        if (isLoad < 0)
                            return;

                        duration = currLabel.duration - extendGraph.distanceMatExtend[currLabel.vertexId][nextVertexId];
                        if (duration < 0)
                            return;

                        arrivalTime = currLabel.serviceTime + extendGraph.distanceMatExtend[currLabel.vertexId][nextVertexId] / extendGraph.velD + extendGraph.sTime;
                        if (arrivalTime > extendGraph.timeWindowExtend.get(nextVertexId)[1])
                            return;


                        serviceTime = Math.max(arrivalTime, extendGraph.timeWindowExtend.get(nextVertexId)[0]);
                        openReq = currLabel.openReq.clone();
                        openReq[nextVertexId - extendGraph.orderNum - 1] = false;
                        servedReq = currLabel.servedReq.clone();
                        servedReq[nextVertexId - extendGraph.orderNum - 1] = true;

                        newLabel = new MyLabel(nextVertexId, cost, isLoad, duration, arrivalTime, serviceTime, openReq, servedReq, currLabel);
                    } else {
                        //什么也不做
                    }
                }
                /*
                Subcase 2.3 : i in V_D
                 */
                else if (1 + extendGraph.orderNum <= currLabel.vertexId && currLabel.vertexId <= extendGraph.orderNum * 2) {
                    // 什么也不做
                } else {
                    System.out.println("!!!!!!!!!error");
                }
            }
        }

        /*
        Case 3 : j in V_S
         */
        else if(1 + extendGraph.orderNum * 2 <= nextVertexId && nextVertexId <= extendGraph.nodeNumExtend - 2) {
            /*
            Subcase 3.1 : i in V_S+{0}
             */
            if (currLabel.vertexId == 0 || (1 + extendGraph.orderNum * 2 <= currLabel.vertexId && currLabel.vertexId <= extendGraph.nodeNumExtend - 2)){
                /*
                Subcase 3.1.1 : c_ij^2 == 1
                 */
                if (extendGraph.c2[currLabel.vertexId][nextVertexId] == 1) {
//                    cost = currLabel.cost;
                    isLoad = currLabel.isLoad;
                    duration = extendGraph.R;
                    arrivalTime = extendGraph.timeWindowExtend.get(nextVertexId)[0];


                    serviceTime = Math.max(arrivalTime, extendGraph.timeWindowExtend.get(nextVertexId)[0]);
                    openReq = currLabel.openReq.clone();
                    servedReq = currLabel.servedReq.clone();

                    newLabel = new MyLabel(nextVertexId, cost, isLoad, duration, arrivalTime, serviceTime, openReq, servedReq , currLabel);
                }

                /*
                Subcase 3.1.2 : c_ij^1 == 1
                 */
                if (extendGraph.c1[currLabel.vertexId][nextVertexId] == 1) {
                    isLoad = currLabel.isLoad;
                    duration = currLabel.duration - extendGraph.distanceMatExtend[currLabel.vertexId][nextVertexId];
                    if (duration < 0)
                        return;

                    arrivalTime = currLabel.serviceTime + extendGraph.distanceMatExtend[currLabel.vertexId][nextVertexId] / extendGraph.velD;
                    if (arrivalTime > extendGraph.timeWindowExtend.get(nextVertexId)[1])
                        return;


                    serviceTime = Math.max(arrivalTime, extendGraph.timeWindowExtend.get(nextVertexId)[0]);
                    openReq = currLabel.openReq.clone();
                    servedReq = currLabel.servedReq.clone();

                    newLabel = new MyLabel(nextVertexId, cost, isLoad, duration, arrivalTime, serviceTime, openReq, servedReq , currLabel);
                }
            }
            /*
            Subcase 3.2 : i in V_P+V_D
            */
            else if(1 <= currLabel.vertexId && currLabel.vertexId <= extendGraph.orderNum * 2){
                isLoad = currLabel.isLoad;
                duration = currLabel.duration - extendGraph.distanceMatExtend[currLabel.vertexId][nextVertexId];
                if (duration < 0)
                    return;

                arrivalTime = currLabel.serviceTime + extendGraph.distanceMatExtend[currLabel.vertexId][nextVertexId] / extendGraph.velD + extendGraph.sTime;
                if (arrivalTime > extendGraph.timeWindowExtend.get(nextVertexId)[1])
                    return;


                serviceTime = Math.max(arrivalTime, extendGraph.timeWindowExtend.get(nextVertexId)[0]);
                openReq = currLabel.openReq.clone();
                servedReq = currLabel.servedReq.clone();

                newLabel = new MyLabel(nextVertexId, cost, isLoad, duration, arrivalTime, serviceTime, openReq, servedReq , currLabel);
            }
        }
        /*
        Case 4 : j = depot2
         */
        else{
            isLoad = currLabel.isLoad;
            if(isLoad != 0)
                return;
            duration = currLabel.duration - extendGraph.distanceMatExtend[currLabel.vertexId][nextVertexId];
            if (duration < 0)
                return;

            arrivalTime = currLabel.serviceTime + extendGraph.distanceMatExtend[currLabel.vertexId][nextVertexId] / extendGraph.velD;
            if (arrivalTime > extendGraph.timeWindowExtend.get(nextVertexId)[1])
                return;


            serviceTime = Math.max(arrivalTime, extendGraph.timeWindowExtend.get(nextVertexId)[0]);
            openReq = currLabel.openReq.clone();
            servedReq = currLabel.servedReq.clone();

            boolean isLeagle = true;
            for (boolean b : openReq) {
                if (b) isLeagle = false;
            }
            boolean allFalse = true;
            for (boolean b : servedReq) {
                if (b) allFalse = false;
            }
            if(allFalse) isLeagle = false;

            if(!isLeagle)
                return;
            newLabel = new MyLabel(nextVertexId, cost, isLoad, duration, arrivalTime, serviceTime, openReq, servedReq , currLabel);
        }

        if (newLabel != null) {
            this.useDominanceRules(newLabel);
        }
    }


    /**
     * 使用支配规则
     * @param labelToCompare 要比较的label
     */
    public void useDominanceRules(MyLabel labelToCompare){
        // 获取new label 的id，只跟id相同的label比较
        int currVertexId = labelToCompare.vertexId;
        ArrayList<MyLabel> processedLabels = labelList.get(currVertexId);

        boolean isDominated = false;

        Iterator<MyLabel> iterator = processedLabels.iterator();
        while (iterator.hasNext()){
            MyLabel other = iterator.next();
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


    /**
     * 重置变量
     */
    protected void reset(){
        this.unprocessedLabels.clear();
        this.labelList.forEach(ArrayList::clear);
        this.shortestPaths.clear();
    }


    /**
     * 更新cost矩阵
     * @param lambda 对偶值
     */
    public void updateCostMatrix(Map<Integer , Double> lambda){
        for (int i = 0; i < vertexNum; i++) {
            for (int j = 0; j < vertexNum; j++) {
                //这里以距离来表示cost，当然也可以换成时间
                extendGraph.cost[i][j] = extendGraph.distanceMatExtend[i][j];
            }
        }

        for(Map.Entry<Integer , Double> entry : lambda.entrySet()){
            for (int j = 0; j < vertexNum; j++) {
                extendGraph.cost[entry.getKey()][j] = extendGraph.cost[entry.getKey()][j] - 0.5*entry.getValue();
                extendGraph.cost[entry.getKey()+extendGraph.orderNum][j] = extendGraph.cost[entry.getKey()+extendGraph.orderNum][j] - 0.5*entry.getValue();
            }
        }

    }


    private static class MyLabelComparator implements Comparator<MyLabel> {
        public int compare(MyLabel first, MyLabel second) {
            if (first.cost < second.cost)
                return -1;
            if (first.cost > second.cost)
                return 1;
            if (first.arrivalTime < second.arrivalTime)
                return -1;
            if (first.arrivalTime > second.arrivalTime)
                return 1;
            if (first.duration > second.duration)
                return -1;
            if (first.duration < second.duration)
                return 1;
            return 0;
        }
    }

    public ArrayList<MyLabel> filtering(ArrayList<MyLabel> allFinalLabels) {
        if (allFinalLabels.isEmpty()) {
            throw new NullPointerException("未找到最短路径");
        }
        allFinalLabels.removeIf(label -> label.cost > - Parameters.EPS);
        return allFinalLabels;
    }

}
