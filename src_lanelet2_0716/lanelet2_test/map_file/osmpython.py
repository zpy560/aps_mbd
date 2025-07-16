import osmnx as ox

# 读取本地osm文件，生成道路网络图
G = ox.graph_from_xml("/home/byd-zpy/lanelet2_ws/src/lanelet2_test/map_file/new_lanelet2_maps_0715.osm")

# 可视化
ox.plot_graph(G)
