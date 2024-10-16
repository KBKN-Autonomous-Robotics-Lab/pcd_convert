# pcd_convert
pcd rotation convert &amp; height segmentation

基本/ros2_ws/srcに入れて/ros2_wsでビルドすれば実行できます。
使用するコマンドを記載しておきます。

livoxデータ取得
ros2 launch livox_to_pointcloud2 livox_to_pointcloud2.launch.py
pcdデータを回転＋高さ調整
ros2 run pcd_convert pcd_rotation
pcdデータを地面とOBS判定
ros2 run pcd_convert pcd_height_segmentation

