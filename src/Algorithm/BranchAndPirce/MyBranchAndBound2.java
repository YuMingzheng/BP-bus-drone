package Algorithm.BranchAndPirce;

import Parameters.ExtendGraph;
import Parameters.Parameters;
import Problem.Route;
import ilog.concert.IloException;

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

    public boolean BBNode(ExtendGraph extendGraph , ArrayList<Route> routes , treeBB branching , ArrayList<Route> bestRoutes , int depth) throws IloException {
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
            bestObj = -1.0;
            bestVal = 0;


        }

    }
}
