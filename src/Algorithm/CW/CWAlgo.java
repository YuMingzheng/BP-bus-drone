package Algorithm.CW;

import Parameters.Data;
import Parameters.ExtendGraph;
import Problem.Node.TimeNode;
import Problem.Route;
import org.json.simple.parser.ParseException;

import javax.script.ScriptException;
import java.io.IOException;
import java.sql.Time;
import java.util.*;

/**
 * @author Yu Mingzheng
 * @date 2024/12/9 15:20
 * @description
 */

public class CWAlgo {
    ExtendGraph extendGraph;

    public CWAlgo(ExtendGraph extendGraph) {
        this.extendGraph = extendGraph;
    }

    public Route[] gen_initial_routes(){
        Route[] initial_routes = new Route[this.extendGraph.orderNum];
        for (int i = 0; i < this.extendGraph.orderNum; i++) {
            Route route = new Route();
            route.throughOrder.add(i+1);
            int[][] visit = new int[][]{
                    {0 , i+1},
                    {i+1 , i+1+this.extendGraph.orderNum},
                    {i+1+this.extendGraph.orderNum , 0}
            };
            for (int j = 0; j < visit.length; j++) {
                double[] start = this.extendGraph.allNode.get(visit[j][0]).getLocation();
                double[] goal  = this.extendGraph.allNode.get(visit[j][1]).getLocation();

                AStar.Result result = AStar.aStar(this.extendGraph.V_S_l , start , goal , this.extendGraph.R , 10);

                for(double[] path_node : result.path){
                    // my indexOf
                    boolean find = false;
                    for (int k = 0; k < this.extendGraph.nodeNum; k++) {
                        if(Arrays.equals(path_node, this.extendGraph.allNode.get(k).getLocation())){
                            route.path.add(k);
                            find = true;
                            break;
                        }
                    }
                    if(!find){System.out.println("my indexOf Exception");}
                }
                route.path.remove(route.path.size()-1);
                route.distance += result.distance;
            }
            route.path.add(this.extendGraph.nodeNum - 1);
            initial_routes[i] = route;
        }

        return initial_routes;
    }

    /**
     *
     * @param input_routes 输入的Route对象的列表
     * @return 返回一个Map，每一个元素对应  key：(index_1 , index_2) --> 节约值
     */
    public LinkedHashMap<List<Integer> , Double> calculate_savings(Route[] input_routes){
        LinkedHashMap<List<Integer> , Double> savings = new LinkedHashMap<>();
        for (int i = 0; i < input_routes.length; i++) {
            for (int j = i+1; j < input_routes.length; j++) {
                double[] D1 = this.extendGraph.allNode.get(input_routes[i].throughOrder.get(input_routes[i].throughOrder.size()-1) + this.extendGraph.orderNum).getLocation();
                double[] P2 = this.extendGraph.allNode.get(input_routes[j].throughOrder.get(input_routes[j].throughOrder.size()-1)).getLocation();
                double[] O  = this.extendGraph.allNode.get(0).getLocation();

                double saving = AStar.aStar(this.extendGraph.V_S_l , D1 , O , this.extendGraph.R , 10).distance +
                        AStar.aStar(this.extendGraph.V_S_l , O , P2 , this.extendGraph.R , 10).distance -
                        AStar.aStar(this.extendGraph.V_S_l , D1 , P2 , this.extendGraph.R , 10).distance;

                savings.put(Arrays.asList(i , j),  saving);
            }
        }

        // 将 LinkedHashMap 转换为 List 并按照值降序排序
        List<Map.Entry<List<Integer>, Double>> list = new ArrayList<>(savings.entrySet());
        list.sort((entry1, entry2) -> Double.compare(entry2.getValue(), entry1.getValue()));

        // 将排序后的 List 转回 LinkedHashMap
        LinkedHashMap<List<Integer>, Double> sortedSavings = new LinkedHashMap<>();
        for (Map.Entry<List<Integer>, Double> entry : list) {
            sortedSavings.put(entry.getKey(), entry.getValue());
        }

        return sortedSavings;
    }

    /**
     *
     * @param origin_route
     */
    public ArrayList<TimeNode> convert_route(Route origin_route){
        ArrayList<TimeNode> extend_route = new ArrayList<>();
        extend_route.add(extendGraph.allNodeExtend.get(0));
        double prev_arrival_time = 0;
        double total_distance = 0;

        for (int route_index = 0; route_index < origin_route.path.size()-1; route_index++) {
            int node_i = origin_route.path.get(route_index);
            int node_j = origin_route.path.get(route_index+1);

            if(extendGraph.connectedBusNetwork[node_i][node_j] == 0 && (1<= node_j && node_j <= extendGraph.orderNum * 2)){
                double node_j_arrival_time = prev_arrival_time + extendGraph.distanceMat[node_i][node_j] / extendGraph.velD;
                TimeNode node_j_extend = ExtendGraph.closest_time_node(extendGraph.allNode.get(node_j), node_j_arrival_time , null , true);
                extend_route.add(node_j_extend);
                prev_arrival_time = node_j_arrival_time;
            }else if(extendGraph.connectedBusNetwork[node_i][node_j] == 0 &&(extendGraph.orderNum * 2 < node_j && node_j < extendGraph.nodeNum - 1)){
                double node_j_arrival_time = prev_arrival_time + extendGraph.distanceMat[node_i][node_j] / extendGraph.velD;
                TimeNode node_j_extend = ExtendGraph.closest_time_node(extendGraph.allNode.get(node_j), node_j_arrival_time , extendGraph.allNode.get(origin_route.path.get(route_index + 2)) , false);

                extend_route.add(node_j_extend);
                prev_arrival_time = node_j_arrival_time;

            }else if(extendGraph.connectedBusNetwork[node_i][node_j] == 1){
                List<TimeNode> neighbors = ExtendGraph.nw.getOrDefault(extend_route.get(extend_route.size()-1), null);
                boolean once_more = true;
                if(neighbors != null) {
                    for (TimeNode neighbor : neighbors) {
                        if (Arrays.equals(neighbor.getLocation(), extendGraph.allNode.get(node_j).getLocation())) {
                            prev_arrival_time = neighbor.getTimeWindow()[0];
                            extend_route.add(neighbor);
                            once_more = false;
                            break;
                        }
                    }
                }
                if(once_more){
                    TimeNode node_i_extend = ExtendGraph.closest_time_node(extendGraph.allNode.get(node_i) , extend_route.get(extend_route.size()-1).getTimeWindow()[0] , extendGraph.allNode.get(origin_route.path.get(route_index + 1)) , false);
                    prev_arrival_time = node_i_extend.getTimeWindow()[0];
                    extend_route.add(node_i_extend);
                    neighbors = ExtendGraph.nw.getOrDefault(extend_route.get(extend_route.size()-1) , null);
                    for (TimeNode neighbor : neighbors) {
                        if(Arrays.equals(neighbor.getLocation(), extendGraph.allNode.get(node_j).getLocation())){
                            prev_arrival_time = neighbor.getTimeWindow()[0];
                            extend_route.add(neighbor);
                            break;
                        }
                    }

                }

            }else if (extendGraph.connectedBusNetwork[node_i][node_j] == 0 && node_j == extendGraph.nodeNum - 1){
                prev_arrival_time = prev_arrival_time + extendGraph.distanceMat[node_i][node_j] / extendGraph.velD;
                extend_route.add(extendGraph.allNodeExtend.get(extendGraph.nodeNumExtend-1));
            }else{
                System.out.println("error");
            }
            total_distance += extendGraph.distanceMat[node_i][node_j];
        }

        return extend_route;

    }

    /**
     *
     * @param input_routes List，输入的Route类型的列表
     * @param index_i 先1后2
     * @param index_j 先1后2
     * @return merged_route : Route ,如果返回null则表示不能merge
     */
    public Route merge(Route[] input_routes , int index_i , int index_j){
        try{
            assert index_i != index_j;

            Route merged_route = new Route();
            merged_route.throughOrder.addAll(input_routes[index_i].throughOrder);
            merged_route.throughOrder.addAll(input_routes[index_j].throughOrder);

            ArrayList<Integer[]> visit = new ArrayList<>();
            for (int i = 0; i < merged_route.throughOrder.size(); i++) {
                int v = merged_route.throughOrder.get(i);
                if(i == 0){
                    visit.add(new Integer[]{0 , v});
                    visit.add(new Integer[]{v , v + extendGraph.orderNum});
                }else{
                    visit.add(new Integer[]{visit.get(visit.size()-1)[1] , v});
                    visit.add(new Integer[]{v , v + extendGraph.orderNum});
                }
            }

            int v = merged_route.throughOrder.get(merged_route.throughOrder.size()-1);
            visit.add(new Integer[]{v + extendGraph.orderNum , 0});


            for (int i = 0; i < visit.size(); i++) {
                double[] start = this.extendGraph.allNode.get(visit.get(i)[0]).getLocation();
                double[] goal  = this.extendGraph.allNode.get(visit.get(i)[1]).getLocation();

                AStar.Result result = AStar.aStar(extendGraph.V_S_l , start , goal,  extendGraph.R , 10);

                for(double[] path_node : result.path){
                    // my indexOf
                    boolean find = false;
                    for (int k = 0; k < this.extendGraph.nodeNum; k++) {
                        if(Arrays.equals(path_node, this.extendGraph.allNode.get(k).getLocation())){
                            merged_route.path.add(k);
                            find = true;
                            break;
                        }
                    }
                    if(!find){System.out.println("my indexOf Exception");}
                }

                merged_route.path.remove(merged_route.path.size()-1);
                merged_route.distance += result.distance;
            }
            merged_route.path.add(this.extendGraph.nodeNum - 1);
            convert_route(merged_route);
            return merged_route;

        }catch (Exception e){
            return null;
        }
    }


    public Route[] cw_algo() throws IOException {
        Route[] routes = gen_initial_routes();

        while(routes.length > this.extendGraph.droneNum){
            Map<List<Integer> ,Double> savings = calculate_savings(routes);
//            System.out.println(savings);

            boolean flag = true;
            for (Map.Entry<List<Integer>, Double> entry : savings.entrySet()) {
                int index_i = entry.getKey().get(0);
                int index_j = entry.getKey().get(1);

                Route i_j_merge = merge(routes, index_i, index_j);

                if(i_j_merge != null){
                    flag = false;
                    Route route_i = routes[index_i];
                    Route route_j = routes[index_j];

                    List<Route> routes_l = new ArrayList<>(Arrays.asList(routes));
                    routes_l.remove(route_i);
                    routes_l.remove(route_j);
                    routes_l.add(i_j_merge);

                    routes = routes_l.toArray(new Route[0]);

                    break;
                }
            }
            if(flag){
                throw new IOException("找不到初始解");
            }
        }



        Route[] routes_return = new Route[routes.length];
        int index = 0;
        for (Route route : routes) {
             ArrayList<TimeNode> route_path_extend = convert_route(route);
             ArrayList<Integer> route_path_extend_index = new ArrayList<>();
            for (TimeNode timeNode : route_path_extend) {
                route_path_extend_index.add(extendGraph.allNodeExtend.indexOf(timeNode));
            }
            route_path_extend_index.remove(route_path_extend_index.size()-1);
            route_path_extend_index.add(extendGraph.nodeNumExtend-1);
            route.path = route_path_extend_index;
            routes_return[index] = route;
            index++;
        }

        return routes_return;
    }

}
