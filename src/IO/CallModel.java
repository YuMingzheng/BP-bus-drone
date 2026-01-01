package IO;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.net.HttpURLConnection;
import java.net.URL;
import java.util.Arrays;

import Parameters.ExtendGraph;
import org.json.JSONObject;
import org.json.simple.parser.ParseException;
import javax.script.ScriptException;

/**
 * @author Yu Mingzheng
 * @date 2024/12/27 15:51
 * @description
 */


public class CallModel {

    public static boolean[] predict(ModelInputData[] modelInputData , double threshold , String modelName) {
        try {
            String url = null;
            switch (modelName) {
                case "lgb":
                    url = "http://localhost:5000/predict-lgb";
                    break;
                case "lr":
                    url = "http://localhost:5000/predict-lr";
                    break;
                case "rf":
                    url = "http://localhost:5000/predict-rf";
                    break;
                case "xgb":
                    url = "http://localhost:5000/predict-xgb";
                    break;
            }

            // 发送POST请求
            HttpURLConnection conn = (HttpURLConnection) new URL(url).openConnection();
            conn.setRequestMethod("POST");
            conn.setRequestProperty("Content-Type", "application/json");
            conn.setDoOutput(true);


            // 请求体
            String jsonInputString = "{\"input\": [" + Arrays.toString(modelInputData) + "]}";
            try (OutputStream os = conn.getOutputStream()) {
                byte[] input = jsonInputString.getBytes("utf-8");
                os.write(input, 0, input.length);
            }

            // 读取响应
            try (BufferedReader br = new BufferedReader(new InputStreamReader(conn.getInputStream(), "utf-8"))) {
                StringBuilder response = new StringBuilder();
                String responseLine;
                while ((responseLine = br.readLine()) != null) {
                    response.append(responseLine.trim());
                }
                JSONObject jsonObject = new JSONObject(response.toString());
                String predictionStr = jsonObject.getString("prediction");


                // 去掉字符串的中括号 "[]"
                predictionStr = predictionStr.substring(1, predictionStr.length() - 1);
                // 按逗号分割字符串，得到一个数组的字符串
                String[] strArray = predictionStr.split(", ");
                // 将每个字符串转换为 double 类型并存入 double 数组
                boolean[] result = new boolean[strArray.length];
                for (int i = 0; i < strArray.length; i++) {
                    result[i] = Double.parseDouble(strArray[i]) > threshold;
                }

                return result;
            }

        } catch (Exception e) {
            e.printStackTrace();
        }
        return new boolean[]{true};
    }

    public static boolean predict(ModelInputData modelInputData) {
        try {
            String url = "http://localhost:5000/predict2";

            // 发送POST请求
            HttpURLConnection conn = (HttpURLConnection) new URL(url).openConnection();
            conn.setRequestMethod("POST");
            conn.setRequestProperty("Content-Type", "application/json");
            conn.setDoOutput(true);


            // 请求体
            String jsonInputString = "{\"input\": [" + modelInputData + "]}";
            try (OutputStream os = conn.getOutputStream()) {
                byte[] input = jsonInputString.getBytes("utf-8");
                os.write(input, 0, input.length);
            }

            // 读取响应
            try (BufferedReader br = new BufferedReader(new InputStreamReader(conn.getInputStream(), "utf-8"))) {
                StringBuilder response = new StringBuilder();
                String responseLine;
                while ((responseLine = br.readLine()) != null) {
                    response.append(responseLine.trim());
                }
                JSONObject jsonObject = new JSONObject(response.toString());
                String predictionStr = jsonObject.getString("prediction");
                double prediction = Double.parseDouble(predictionStr);


                // 根据预测值判断是否满足条件
                return prediction > 1e-5;
            }

        } catch (Exception e) {
            e.printStackTrace();
        }
        return true;
    }


    public static void main(String[] args) throws ScriptException, IOException, ParseException {

//        double start = System.currentTimeMillis();
//        ModelInputData[] modelInputData = new ModelInputData[1000];
//        for (int i = 0; i < modelInputData.length; i++) {
//            modelInputData[i] = new ModelInputData();
//        }
//        CallModel.predict(modelInputData , -1 , "rf");
//        System.out.println("Method 1 : Time consumption: " + (System.currentTimeMillis() - start) / 1000.0);


//        start = System.currentTimeMillis();
//        for (int i = 0; i < modelInputData.length; i++) {
//            CallModel.predict(new ModelInputData());
//        }
//
//        System.out.println("Method 2 : Time consumption: " + (System.currentTimeMillis() - start) / 1000.0);

        double start = System.currentTimeMillis();
        String instanceName = "50_1_1_1_36323";
//            String path = "D:\\硕士毕设\\bus+drone\\bus-drone-code\\data\\gurobi_algo_instance\\50_1_1_1_8545.json";
        String path = "D:\\硕士毕设\\bus+drone\\bus-drone-code\\data\\choose_model_instance\\" + instanceName + ".json";
//            ExtendGraph extendGraph = new ExtendGraph(path , true);
        ExtendGraph extendGraph = new ExtendGraph(path , 0.2, "rf");
        System.out.println("Method 1 : Time consumption: " + (System.currentTimeMillis() - start) / 1000.0);


    }
}
