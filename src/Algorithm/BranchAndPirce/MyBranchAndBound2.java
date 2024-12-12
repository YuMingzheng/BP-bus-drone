package Algorithm.BranchAndPirce;

import Parameters.ExtendGraph;
import Parameters.Parameters;
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
public class MyBranchAndBound2 {
    double lowerBound;
    double upperBound;

    public MyBranchAndBound2() {
        lowerBound = -1E10;
        upperBound = 1E10;
    }

    class treeBB{
        // 使用linked tree list，记录BB过程中所有的branching
        treeBB father;     // 链接到分支之前处理的节点
        treeBB son0;       // 链接到树左侧的子节点(edge=0；第一次处理)=>需要它来计算全局下限
        int branchOrder1;    // branch on order【在left分支中两个branch order必须served by the same drone】
        int branchOrder2;    //                【在right分支中branch order必须served by different drones】
        int branchValue;     //                【value of the branching (remove edge=0; set edge=1)】
        double lowestValue; // lower bound on the solution if we start from this node (i.e. looking only down for this tree)
        boolean topLevel;  // 要计算全局下限，需要知道上面的所有内容是否已经考虑过
    }

    public void OrderBasedOnBranching(ExtendGraph extendGraph , treeBB branching , boolean recur){
        int i;
    }

    public boolean BBNode(ExtendGraph extendGraph , ArrayList<Route> routes , treeBB branching , ArrayList<Route> bestRoutes , int depth) throws IloException, IOException {
        int i , j , selectedOrder1 , selectedOrder2 , prevCity , city , bestVal;
        double coef , bestObj , change , CGobj;
        boolean feasible ;

        /**
         * (1) 首先判断上下界是否满足达到制定的gap。如果是，则认为已经完成了搜索，跳出递归
         */

        System.out.println("Upperbound : " + upperBound + " Lowerbound : " + lowerBound);
        if((upperBound - lowerBound) / upperBound <= Parameters.gap){
            return true;
        }

        /**
         * (2) 分支节点为null时候，说明为初始状态，那么分配根节点root，开始进行分支过程
         */
        if(branching == null){
            treeBB newNode = new treeBB();
            newNode.father = null;
            newNode.topLevel = true;
            newNode.branchOrder1 = -1;
            newNode.branchOrder2 = -1;
            newNode.branchValue = -1;
            newNode.son0 = null;
            branching = newNode;
        }

        /**
         * (3) branchValue < 1时候，两个order必须same，否则different
         */
        if (branching.branchValue < 1){
            System.out.println("\nEdge " + branching.branchOrder1 + " and " + branching.branchOrder2 + ": same");
        }else{
            System.out.println("\nEdge " + branching.branchOrder1 + " and " + branching.branchOrder2 + ": different");
        }

        /**
         * (4) 利用column generation计算该节点的lower bound，注意上面有order的处理
         */
        MyColumnGen CG = new MyColumnGen();
        CGobj = CG.computeColGen(extendGraph , routes);

        if((CGobj > Parameters.bigM) || (CGobj < -Parameters.EPS)){
            System.out.println("Relax Infeasible | Lower bound : " + lowerBound + " Upper bound : " + upperBound + " | BB depth : " + depth);
            return true;
        }
        branching.lowestValue = CGobj;

        /**
         * (5) 新的lower bound计算得到后，比较要不要更新全局的lower bound
         */
        if((branching.father != null) && (branching.father.son0 != null) && branching.father.father.topLevel){
            lowerBound = Math.min(branching.lowestValue, branching.father.son0.lowestValue);
            branching.topLevel = true;
        }else if(branching.father == null){
            lowerBound = CGobj;
        }

        /**
         * (6) 接下来是常规操作，判断lower bound和upper bound的大小，进行剪枝或者再次分支。
         *     如果该分支的lower bound > upper bound，则cut掉
         */
        if (branching.lowestValue > upperBound) {
            CG = null;
            System.out.println("CUT | Lower bound: " + lowerBound + " | Upper bound: " + upperBound + " | Gap: " + ((upperBound - lowerBound) / upperBound) + " | BB Depth: " + depth + " | Local CG cost: " + CGobj + " | " + routes.size() + " routes");
            return true;

        /**
         * (7) 否则，先判断column generation找到的解是不是整数解（所有的边不能有小数）
         */
        }else{
            feasible = true;

            selectedOrder1 = -1;
            selectedOrder2 = -1;

            bestVal = 0;

            for (i = 0; i < extendGraph.orderNum; i++) {
                Arrays.fill(extendGraph.twoOrder[i] , 0);
            }

            for(Route r : routes){
                int[] throughOrder = r.throughOrder.stream().mapToInt(k->k).toArray();
                for (int k = 0; k < throughOrder.length-1; k++) {
                    for (int l = k+1; l < throughOrder.length; l++) {
                        extendGraph.twoOrder[throughOrder[k]][throughOrder[l]]+=r.getQ();
                    }
                }
            }
            for ( i = 0; i < extendGraph.twoOrder.length; i++) {
                for ( j = i+1; j < extendGraph.twoOrder.length; j++) {
                    if(extendGraph.twoOrder[i][j]>0.5) extendGraph.twoOrder[i][j]-=0.5;
                    else extendGraph.twoOrder[i][j]=0.5-extendGraph.twoOrder[i][j];
                }
            }
            double minValue = extendGraph.twoOrder[0][1];
            int row=0,column=1;
            for( i=0;i<extendGraph.twoOrder.length;i++){
                for( j=i+1;j<extendGraph.twoOrder.length;j++){
                    if(extendGraph.twoOrder[i][j]<minValue){
                        minValue=extendGraph.twoOrder[i][j];
                        row=i;
                        column=j;
                    }
                }
            }
            selectedOrder1 = row+1;
            selectedOrder2 = column+1;


            for(Route r : routes){
                if ((r.getQ() > 1e-6) && ((r.getQ() < 1 - 1e-6) || (r.getQ() > 1 + 1e-6))) {
                    feasible = false;
                    break;
                }
            }


            /**
             * (8) 如果是整数解，那么就找到了一个可行解，判断要不要更新upper lound
             */
            if(feasible){
                if(branching.lowestValue < upperBound){
                    upperBound = branching.lowestValue;
                    bestRoutes.clear();
                    for (Route r : routes) {
                        if(r.getQ() > Parameters.EPS){
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
             * (9) 否则，找一条边继续进行分支
             *     这里是先分左支，即selected的两个order要被同一个drone配送，要移除column generation的RLMP中违反该规则的路径：
             */
            }else{
                System.out.println("INTEG INFEAS | Lower bound: " + lowerBound + " | Upper bound: " + upperBound + " | Gap: " + ((upperBound - lowerBound) / upperBound) + " | BB Depth: " + depth + " | Local CG cost: " + CGobj + " | " + routes.size() + " routes");
                System.out.flush();

                //////////////
                // branching
                treeBB newNode1 = new treeBB();
                newNode1.father = branching;
                newNode1.branchOrder1 = selectedOrder1;
                newNode1.branchOrder2 = selectedOrder2;
                newNode1.branchValue = bestVal; //TODO 没设置bestVal
                newNode1.lowestValue = -1E10;
                newNode1.son0 = null;

                // 筛选一下边，去掉不符合左分支的边
                ArrayList<Route> nodeRoutes = new ArrayList<>();
                for(Route r : routes){
                    ArrayList<Integer> throughOrder = r.throughOrder;
                    boolean accept = throughOrder.contains(selectedOrder1) == throughOrder.contains(selectedOrder2);

                    if(accept){
                        nodeRoutes.add(r);
                    }
                }
                boolean ok;
                ok = BBNode(extendGraph , nodeRoutes , newNode1 , bestRoutes , depth+1);
                if(!ok)
                    return false;

                branching.son0 = newNode1;


                /**
                 * (10) 然后是右分支，该分支限定两个order被不同drone配送
                 */
                treeBB newNode2 = new treeBB();
                newNode2.father = branching;
                newNode2.branchOrder1 = selectedOrder1;
                newNode2.branchOrder2 = selectedOrder2;
                newNode2.branchValue = 1 - bestVal;
                newNode2.lowestValue = -1E10;
                newNode2.son0 = null;


            }


        }


    return false;
    }
}
