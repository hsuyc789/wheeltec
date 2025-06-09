# 完成的項目
## 複製 object_tracking_YOLO.py 到 object_tracking_YOLO_soccer.py 
將來使用 object_tracking_YOLO_soccer.py 當作影像辨識的節點
* 新增 2 個 publish nodes `Visual_ball_position_x`, `Visual_ball_width`
* `Visual_ball_position_x` 用於回傳最大 confidence ball 的中心 x 座標
* `Visual_ball_width` 用於回傳最大 confidence ball 的寬度

## 複製 robot_task.py 到 soccer.py
將來使用 soccer.py 當作機器人主程式的節點
在 `soccer.py` 中:
* 刪除關於 laser 的程式和 variable
* 新增一個 Ball class，用於紀錄 ball 目前位置、距離等

# 待完成的事項
* 調整計算ball 距離的線性算法常數(斜率、C)
* 對 x 位置製作 PID controller(可參考或修改之前 visual 的程式碼)
* 關於球門的辨識邏輯
* 使用加速度感測器和gyro紀錄目前的位置和轉向角度(也可以用 ros2 nav 等方法)
* 踢球的邏輯