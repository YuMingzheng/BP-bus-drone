package Parameters;

import java.io.FileReader;
import java.io.IOException;
import java.util.*;
import javax.script.ScriptEngine;
import javax.script.ScriptEngineManager;
import javax.script.ScriptException;

import Problem.Node.Node;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;


/**
 * @author Yu Mingzheng
 * @date 2024/9/13 09:57
 * @description
 */
public class Data {
    // 关键数据变量
    public double T;
    public int droneNum;
    public int orderNum;
    public int lineNum;
    public double busIntv;
    public double velB;
    public double velD;
    public double R;
    public double resversRunIntv;
    public Node depot;
    public Node depot2;
    public double[][][] a;
    public List<double[]> timeWindow;
    public List<List<double[]>> V_S_l;
    public double stopTime;

    public Data(String filePath) throws IOException, ParseException, ScriptException {
        JSONParser parser = new JSONParser();
        JSONObject jsonObj = (JSONObject) parser.parse(new FileReader(filePath));

        ScriptEngineManager mgr = new ScriptEngineManager();
        ScriptEngine engine = mgr.getEngineByName("JavaScript");

//        T = parseDouble(jsonObj.get("T"), engine);
//        droneNum = parseInt(jsonObj.get("drone_num"), engine);
//        orderNum = parseInt(jsonObj.get("order_num"), engine);
//        lineNum = parseInt(jsonObj.get("line_num"), engine);
//        busIntv = parseDouble(jsonObj.get("bus_intv"), engine);
//        velB = parseDouble(jsonObj.get("vel_b"), engine);
//        velD = parseDouble(jsonObj.get("vel_d"), engine);
//        R = parseDouble(jsonObj.get("R"), engine);
//        resversRunIntv = parseDouble(jsonObj.get("resvers_runn_intv"), engine);
//        stopTime = parseDouble(jsonObj.get("stop_time"), engine);

        T = Double.parseDouble((String) jsonObj.get("T"));
        droneNum = Integer.parseInt((String) jsonObj.get("drone_num"));
        orderNum = Integer.parseInt((String) jsonObj.get("order_num"));
        lineNum = Integer.parseInt(((String) jsonObj.get("line_num")).split(" ")[0]) * 2;
        busIntv = Double.parseDouble((String) jsonObj.get("bus_intv"));
        velB = Double.parseDouble((String) jsonObj.get("vel_b"));
        velD = Double.parseDouble((String) jsonObj.get("vel_d"));
        R = Double.parseDouble((String) jsonObj.get("R"));
        resversRunIntv = Double.parseDouble((String) jsonObj.get("resvers_runn_intv"));
        stopTime = Double.parseDouble((String) jsonObj.get("stop_time"));

        depot = new Node((String) jsonObj.get("depot"));
        depot2 = new Node((String) jsonObj.get("depot2"));

        a = parse3DArray((String) jsonObj.get("a"));

        timeWindow = parse2DArray((String) jsonObj.get("time_window"));

        V_S_l = parse3DList((String) jsonObj.get("V_S_l"));


    }

    private int parseInt(Object obj, ScriptEngine engine) throws ScriptException {
        String str = obj.toString().replace("\"", "").trim();
        return ((Number) engine.eval(str)).intValue();
    }


    private double parseDouble(Object obj, ScriptEngine engine) throws ScriptException {
        String str = obj.toString().replace("\"", "").trim();
        return ((Number) engine.eval(str)).doubleValue();
    }


    private double[][][] parse3DArray(String str) throws ScriptException {
        // 解析字符串，如 "np.array([[[20, 70], [75, 50]]])"
        str = str.replace("np.array(", "").replace(")", "").trim();
        return parseJSONArray3D(str);
    }

    private double[][][] parseJSONArray3D(String str) {
        JSONArray jsonArray = parseJSONArray(str);
        double[][][] result = new double[jsonArray.size()][][];
        for (int i = 0; i < jsonArray.size(); i++) {
            JSONArray innerArray = (JSONArray) jsonArray.get(i);
            result[i] = new double[innerArray.size()][];
            for (int j = 0; j < innerArray.size(); j++) {
                JSONArray innerInnerArray = (JSONArray) innerArray.get(j);
                result[i][j] = new double[innerInnerArray.size()];
                for (int k = 0; k < innerInnerArray.size(); k++) {
                    result[i][j][k] = ((Number) innerInnerArray.get(k)).doubleValue();
                }
            }
        }
        return result;
    }

    private List<double[]> parse2DArray(String str) {
        // 解析字符串，如 "[[20, 25], [35, 40]]"
        JSONArray jsonArray = parseJSONArray(str);
        List<double[]> result = new ArrayList<>();
        for (Object obj : jsonArray) {
            JSONArray innerArray = (JSONArray) obj;
            double[] arr = new double[innerArray.size()];
            for (int i = 0; i < innerArray.size(); i++) {
                arr[i] = ((Number) innerArray.get(i)).doubleValue();
            }
            result.add(arr);
        }
        return result;
    }

    private List<List<double[]>> parse3DList(String str) {
        // 解析字符串，如 "[[[30, 70],[40, 50],[60, 50],[70, 30]], ...]"
        JSONArray jsonArray = parseJSONArray(str);
        List<List<double[]>> result = new ArrayList<>();
        for (Object obj : jsonArray) {
            JSONArray innerArray = (JSONArray) obj;
            List<double[]> innerList = new ArrayList<>();
            for (Object innerObj : innerArray) {
                JSONArray pointArray = (JSONArray) innerObj;
                double[] point = new double[pointArray.size()];
                for (int i = 0; i < pointArray.size(); i++) {
                    point[i] = ((Number) pointArray.get(i)).doubleValue();
                }
                innerList.add(point);
            }
            result.add(innerList);
        }
        return result;
    }

    private JSONArray parseJSONArray(String str) {
        // 将字符串解析为JSONArray
        JSONParser parser = new JSONParser();
        try {
            return (JSONArray) parser.parse(str);
        } catch (ParseException e) {
            e.printStackTrace();
            return new JSONArray();
        }
    }

}

