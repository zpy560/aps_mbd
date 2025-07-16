import lanelet2
import lanelet2.io
import lanelet2.projection

# 1. 指定你的osm文件路径
osm_file = "/home/byd-zpy/lanelet2_ws/src/Lanelet2/new_lanelet2_maps_z0.osm"

# 2. 选择投影方式（本地坐标用LocalCartesian，地理坐标用UTM或其他）
proj = lanelet2.projection.LocalCartesian()

# 3. 加载地图
lanelet_map = lanelet2.io.load(osm_file, proj)

print("地图导入成功，lanelet数量：", len(lanelet_map.laneletLayer))