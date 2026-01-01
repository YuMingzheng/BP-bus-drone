package Algorithm.BranchAndPirce;

import Parameters.ExtendGraph;
import Old.Parameters;
import Problem.Route;
import ilog.concert.IloException;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;

/**
 * @author Yu Mingzheng
 * @date 2024/10/30 16:47
 * @description
 */
public class MyBranchAndBound {
    double lowerBound;
    double upperBound;

    public MyBranchAndBound() {
        lowerBound = -1E10;
        upperBound = 1E10;
    }

    class treeBB{
        // 使用linked tree list，记录BB过程中所有的branching
        treeBB father;     // 链接到分支之前处理的节点
        treeBB son0;       // 链接到树左侧的子节点(edge=0；第一次处理)=>需要它来计算全局下限
        int branchFrom;    // branch on edges between cities =>【city origin of the edge】
        int branchTo;      // we branch on edges between cities => 【city destination of the edge】
        int branchValue;   // we branch on edges between cities => 【value of the branching (remove edge=0; set edge=1)】
        double lowestValue;// lower bound on the solution if we start from this node (i.e. looking only down for this tree)
        boolean topLevel;  // 要计算全局下限，需要知道上面的所有内容是否已经考虑过
    }

    public void EdgesBasedOnBranching(ExtendGraph extendGraph , treeBB branching , boolean recur){
        int i ;
        if(branching.father != null){
            if(branching.branchValue == 0){ // 禁止这个edge
                extendGraph.distanceMatExtendChange[branching.branchFrom][branching.branchTo] = Parameters.bigM;
            }else{
                for (i = 0; i < branching.branchTo; i++)
                    extendGraph.distanceMatExtendChange[branching.branchFrom][i] = Parameters.bigM;
                for(i++;i < extendGraph.nodeNumExtend ; i++)
                    extendGraph.distanceMatExtendChange[branching.branchFrom][i] = Parameters.bigM;

                if(branching.branchTo != extendGraph.nodeNumExtend-1){
                    for(i = 0 ; i < branching.branchFrom ; i++)
                        extendGraph.distanceMatExtendChange[i][branching.branchTo] = Parameters.bigM;
                    for(i++;i < extendGraph.nodeNumExtend ; i++)
                        extendGraph.distanceMatExtendChange[i][branching.branchTo] = Parameters.bigM;
                }
                extendGraph.distanceMatExtendChange[branching.branchTo][branching.branchFrom] = Parameters.bigM;
            }
            if(recur)
                EdgesBasedOnBranching(extendGraph , branching.father , true);
        }
    }


    public boolean BBNode(ExtendGraph extendGraph , ArrayList<Route> routes ,  treeBB branching, ArrayList<Route> bestRoutes, int depth) throws IloException, IOException {
        int i , j , bestEdge1 , bestEdge2 , prevcity , city , bestVal;
        double coef , bestObj , change , CGobj ;
        boolean feasible;

        /**
         *  (1) 首先判断上下界是否满足达到制定的gap。如果是，则认为已经完成了搜索，跳出递归
         */
        System.out.println("Upperbound : " + upperBound + " Lowerbound : " + lowerBound);
        if((upperBound - lowerBound) / upperBound <= Parameters.gap){
            return true;
        }

        /**
         *  (2) 分支节点为null时候，说明为初始状态，那么分配根节点root，开始进行分支过程
         */
        if (branching == null){
            treeBB newNode = new treeBB();
            newNode.father = null;
            newNode.topLevel = true;
            newNode.branchFrom = -1;
            newNode.branchTo = -1;
            newNode.branchValue = -1;
            newNode.son0 = null;
            branching = newNode;
        }

        /**
         *  (3) branchValue < 1时候，该边被禁止，否则该边被选择（一定要通过）
         */
        if (branching.branchValue < 1){
            System.out.println("\nEdge from " + branching.branchFrom + " to " + branching.branchTo + ": forbid");
        }else{
            System.out.println("\nEdge from " + branching.branchFrom + " to " + branching.branchTo + ": chosen");
        }

        /**
         *  (4) 利用column generation计算该节点的lower bound，注意上面有边被禁止/选择了
         *      ，因此这些边在cg中是无法被访问或者要强制经过的
         */
        MyColumnGen CG = new MyColumnGen();
        CGobj = CG.computeColGen(extendGraph , routes);

        if((CGobj > Parameters.bigM) || (CGobj < -Parameters.EPS)){
            System.out.println("Relax Infeasible | Lower bound : " + lowerBound + " Upper bound : " + upperBound + " | BB depth : " + depth);
            return true;
        }
        branching.lowestValue = CGobj;

        /**
         *  (5) 新的lower bound计算得到后，比较要不要更新全局的lower bound
         */
        if((branching.father != null) && (branching.father.son0 != null) && branching.father.father.topLevel){
            lowerBound = Math.min(branching.lowestValue, branching.father.son0.lowestValue);
            branching.topLevel = true;
        }else if(branching.father == null){
            lowerBound = CGobj;
        }

        /**
         *  (6) 接下来是常规操作，判断lower bound和upper bound的大小，进行剪枝或者再次分支。
         *      如果该分支的lower bound > upper bound，则cut掉
         */
        if (branching.lowestValue > upperBound){
            CG = null;
            System.out.println("CUT | Lower bound: " + lowerBound + " | Upper bound: " + upperBound + " | Gap: " + ((upperBound - lowerBound) / upperBound) + " | BB Depth: " + depth + " | Local CG cost: " + CGobj + " | " + routes.size() + " routes");
            return true;


        /**
         *  (7) 否则，先判断column generation找到的解是不是整数解（所有的边不能有小数）
         */
        }else {
            feasible = true;
            bestEdge1 = -1;
            bestEdge2 = -1;
            bestObj = -1.0;
            bestVal = 0;

            // 将path var转为edge var
            for (i = 0; i < extendGraph.nodeNumExtend; i++) {
                Arrays.fill(extendGraph.edges[i], 0.0);
            }
            for (Route r : routes) {
                if (r.getQ() > Parameters.EPS) {
                    ArrayList<Integer> path = r.getPath();
                    prevcity = 0;
                    for (i = 1; i < path.size(); i++) {
                        city = path.get(i);
                        extendGraph.edges[prevcity][city] += r.getQ();
                        prevcity = city;
                    }
                }
            }

            // 找到一个分数（fractional）的edge
            for (i = 0; i < extendGraph.nodeNumExtend; i++) {
                for (j = 0; j < extendGraph.nodeNumExtend; j++) {
                    if((1<=i && i <= extendGraph.orderNum*2) ||(1<=j && j <= extendGraph.orderNum*2)) {
                        coef = extendGraph.edges[i][j];
                        if ((coef > 1e-6) && ((coef < 1 - 1e-6) || (coef > 1 + 1e-6))) {
                            feasible = false;
                            // what if we impose this route in the solution? Q=1
                            // keep the ref of the edge which should lead to the largest change
                            //                        change = Math.min(coef, Math.abs(1.0 - coef));
                            //                        change *= routes.get(i).getCost();
                            //                        if (change > bestObj) {
                            bestEdge1 = i;
                            bestEdge2 = j;
                            //                            bestObj = change;
                            bestVal = (Math.abs(1.0 - coef) > coef) ? 0 : 1;
                        }
                    }
                }
            }

            /**
             *  (8) 如果是整数解，那么就找到了一个可行解，判断要不要更新upper lound
             */
            if (feasible) {
                if (branching.lowestValue < upperBound) {
                    upperBound = branching.lowestValue;
                    bestRoutes.clear();
                    for (Route r : routes) {
                        if (r.getQ() > Parameters.EPS) {
                            Route optim = new Route();
                            optim.setDistance(r.getDistance());
                            optim.path = r.getPath();
                            optim.setQ(r.getQ());
                            bestRoutes.add(optim);
                        }
                    }
                    System.out.println("OPT | Lower bound: " + lowerBound + " | Upper bound: " + upperBound + " | Gap: " + ((upperBound - lowerBound) / upperBound) + " | BB Depth: " + depth + " | Local CG cost: " + CGobj + " | " + routes.size() + " routes");
                    System.out.flush();
                }else{
                    System.out.println("FEAS | Lower bound: " + lowerBound + " | Upper bound: " + upperBound + " | Gap: " + ((upperBound - lowerBound) / upperBound) + " | BB Depth: " + depth + " | Local CG cost: " + CGobj + " | " + routes.size() + " routes");
                }
                return true;

            /**
             *  (9) 否则，找一条边继续进行分支（这条边具体的选择也会影响分支的速度，选择看步骤(7）中的 bestEdge1和 bestEdge2)
             *  ，EdgesBasedOnBranching函数的作用是通过设置距离矩阵各边的距离
             * 	，来禁止或者指定选择一些边（比如将一条边的距离设置为正无穷，那么该边就无法访问了）
             * 	。这里是先分左支，即该边被禁止，要移除column generation的RLMP中包含该边的所有路径：
             */
            }else{
                System.out.println("INTEG INFEAS | Lower bound: " + lowerBound + " | Upper bound: " + upperBound + " | Gap: " + ((upperBound - lowerBound) / upperBound) + " | BB Depth: " + depth + " | Local CG cost: " + CGobj + " | " + routes.size() + " routes");
                System.out.flush();
                //////////////
                // branching
                // first branch -> 设置 edges[bestEdge1][bestEdge2] = 0
                // 在tree list中记录branching的信息
                treeBB newNode1 = new treeBB();
                newNode1.father = branching;
                newNode1.branchFrom = bestEdge1;
                newNode1.branchTo = bestEdge2;
                newNode1.branchValue = bestVal;
                newNode1.lowestValue = -1E10;
                newNode1.son0 = null;

                //  branching on edges[bestEdge1][bestEdge2]=0
                EdgesBasedOnBranching(extendGraph, newNode1, false);

                // the initial lp for the CG contains all the routes of the previous
                // solution less(去掉分支的边) the routes containing this arc
                ArrayList<Route> nodeRoutes = new ArrayList<>();
                for (Route r : routes) {
                    ArrayList<Integer> path = r.getPath();
                    boolean accept = true;
                    if(path.size() > 3){ // TODO 大于3是为了保证不去掉Depot-City-Depot的边，来保证可行性
                        prevcity = 0;
                        for(j = 1; accept && (j < path.size()) ; j++){
                            city = path.get(j);
                            if((prevcity == bestEdge1) & (city == bestEdge2))
                                accept = false;
                            prevcity = city;
                        }
                    }
                    if(accept){
                        nodeRoutes.add(r);
                    }
                }
                boolean ok;
                ok = BBNode(extendGraph , nodeRoutes, newNode1, bestRoutes, depth +1);
                if(!ok){
                    return false;
                }

                branching.son0 = newNode1;

                /**
                 *  (10) 然后是右分支，该分支限定该边一定要经过，因此要移除掉columns generation的RLMP中不包含该边的所有路径
                 */
                // second branch -> set edges[bestEdge1][bestEdge2]=1
                treeBB newNode2 = new treeBB();
                newNode2.father = branching;
                newNode2.branchFrom = bestEdge1;
                newNode2.branchTo = bestEdge2;
                newNode2.branchValue = 1 - bestVal;
                newNode2.lowestValue = -1E10;
                newNode2.son0 = null;

                // branching on edges[bestEdge1][bestEdge2] = 1
                // 需要重新更新dist matrix
                for(i = 0; i < extendGraph.nodeNumExtend; i++) {
                    System.arraycopy(extendGraph.distanceMatExtend[i] , 0 , extendGraph.distanceMatExtendChange[i],0,extendGraph.nodeNumExtend);
                }
                // reinitialize了 因此需要recur递归
                EdgesBasedOnBranching(extendGraph, newNode2, true);

                ArrayList<Route> nodeRoutes2 = new ArrayList<>();
                for (Route r : routes) {
                    ArrayList<Integer> path = r.getPath();
                    boolean accept = true;
                    if(path.size() > 3){
                        prevcity = 0;
                        for(i = 1; accept && (i < path.size()) ;i++){
                            city = path.get(i);
                            if(extendGraph.distanceMatExtendChange[prevcity][city] >= Parameters.bigM-1e-6){
                                accept = false;
                            }
                            prevcity = city;
                        }
                    }
                    if(accept){
                        nodeRoutes2.add(r);
                    }
                }
                ok = BBNode(extendGraph , nodeRoutes2, newNode2, bestRoutes, depth +1);
                 branching.lowestValue = Math.min(newNode1.lowestValue, newNode2.lowestValue);
                 return ok;
            }
        }

    }
}
