#include <lanelet2_io/Io.h>
#include <lanelet2_projection/LocalCartesian.h>
#include <lanelet2_core/LaneletMap.h>
#include <iostream>
#include <lanelet2_core/geometry/impl/Lanelet.h>
#include "lanelet2_core/Forward.h"
#include <lanelet2_routing/RoutingCost.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_routing/Route.h>
#include <lanelet2_core/Attribute.h>
// #include <lanelet2_core/Forward.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <chrono>
int main()
{
    int num = 0;
    // 1. 指定osm文件路径
    std::string osm_file = "/home/byd-zpy/lanelet2_ws/src/lanelet2_test/map_file/new_lanelet2_maps_0715.osm";
    // std::string osm_file = "/home/byd-zpy/lanelet2_ws/src/lanelet2_test/map_file/map.osm";
    // std::string osm_file = "/home/byd-zpy/lanelet2_ws/src/lanelet2_test/map_file/byd_road.osm";
    lanelet::GPSPoint origin_gps{22.6881902, 114.3480115, 0.0};
    lanelet::Origin origin(origin_gps); // 例如杭州
    lanelet::projection::LocalCartesianProjector projector(origin);
    int i = 0;
    // 3. 读取地图
    lanelet::ErrorMessages errors;
    lanelet::LaneletMapPtr map = lanelet::load(osm_file, projector, &errors);
    auto &all_lanelets = map->laneletLayer;
    std::cout << "地图导入成功,lanelet数量: " << all_lanelets.size() << std::endl;

    // 3. 遍历Lanelet，打印ID与长度
    for (const auto &lanelet : all_lanelets)
    {
        i++;
        auto length = lanelet::geometry::length2d(lanelet);
        std::cout << i << " :" << "Lanelet ID: " << lanelet.id() << ", length: " << length << " meters\n";
    }

    // 保存所有元素到各自的数组
    std::vector<lanelet::Lanelet> lanelets(map->laneletLayer.begin(), map->laneletLayer.end());
    std::vector<lanelet::Point3d> points(map->pointLayer.begin(), map->pointLayer.end());
    std::vector<lanelet::LineString3d> linestrings(map->lineStringLayer.begin(), map->lineStringLayer.end());
    std::vector<lanelet::Polygon3d> polygons(map->polygonLayer.begin(), map->polygonLayer.end());
    std::vector<lanelet::Area> areas(map->areaLayer.begin(), map->areaLayer.end());

    std::cout << "Lanelet数量: " << lanelets.size() << std::endl;
    std::cout << "Point数量: " << points.size() << std::endl;
    std::cout << "LineString数量: " << linestrings.size() << std::endl;
    std::cout << "Polygon数量: " << polygons.size() << std::endl;
    std::cout << "Area数量: " << areas.size() << std::endl;
    // for (const auto &lanelet : lanelets)
    // {
    //     // std::cout << "Lanelet ID: " << lanelet.id() << std::endl;
    //     // std::cout << ", LeftBound Points:" << std::endl;
    //     // num = 0;
    //     // for (const auto &pt : lanelet.leftBound())
    //     // {
    //     //     num++;
    //     //     std::cout << num << "个点:(" << pt.x() << ", " << pt.y() << ", " << pt.z() << ") " << std::endl;
    //     // }
    //     // std::cout << "RightBound Points:" << std::endl;
    //     // num = 0;
    //     // for (const auto &pt : lanelet.rightBound())
    //     // {
    //     //     num++;
    //     //     std::cout << num << "个点:(" << pt.x() << ", " << pt.y() << ", " << pt.z() << ") " << std::endl;
    //     // }
    //     // std::cout << std::endl;
    //     // std::cout << "lanelet.centerline:" << std::endl;
    //     num = 0;
    //     for (const auto &pt : lanelet.centerline())
    //     {
    //         num++;
    //         std::cout << num << "个点:" << pt.x() << ", " << pt.y() << ", " << pt.z() << std::endl;
    //     }
    //     // std::cout << std::endl;
    // }
    std::cout << std::endl;
    num = 0;
    auto start_pt = points[0];
    auto end_pt = points[1];
    for (const auto &point : points)
    {
        num = num + 1;
        // std::cout << "Point ID: " << point.id() << ", Coordinates: " << std::endl;
        std::cout << "Point ID: " << point.id() << "个点:(" << point.x() << ", " << point.y() << ", " << point.z() << ") " << std::endl;
        // std::cout << std::endl;
        if (853 == point.id())
        {
            end_pt = point;
        }
        if (598 == point.id())
        {
            start_pt = point;
        }
        
    }

    // for (const auto &linestring : linestrings)
    // {
    //     std::cout << "LineString ID: " << linestring.id() << ", Points: " << std::endl;
    //     num = 0;
    //     // 遍历LineString中的点
    //     for (auto it = linestring.begin(); it != linestring.end(); ++it)
    //     {
    //         num = num + 1;
    //         std::cout << num << "个点:(" << it->x() << ", " << it->y() << ", " << it->z() << ") " << std::endl;
    //         if (72783.9 == it->x())
    //         {
    //             std::cout << std::endl;
    //             std::cout << std::endl;
    //             std::cout << "找到目标点:(" << it->x() << ", " << it->y() << ", " << it->z() << ")" << std::endl;
    //             // std::cout << "Point ID: " << point.id() << std::endl;
    //             std::cout << std::endl;
    //             std::cout << std::endl;
    //         }
    //     }
    //     std::cout << std::endl;
    // }

    // for (const auto &polygon : polygons)
    // {
    //     std::cout << "Polygon ID: " << polygon.id() << ", Points: " << std::endl;
    //     num = 0;
    //     for (const auto &point : polygon)
    //     {
    //         num = num + 1;
    //         std::cout << num << "个点:(" << point.x() << ", " << point.y() << ", " << point.z() << ") " << std::endl;
    //     }
    //     std::cout << std::endl;
    // }

    // for (const auto &area : areas)
    // {
    //     std::cout << "Area ID: " << area.id() << ", Polygons: ";
    //     num = 0;
    //     for (const auto &linestring : area.outerBound())
    //     {
    //         for (auto it = linestring.begin(); it != linestring.end(); ++it)
    //         {
    //             num = num + 1;
    //             std::cout << num << "个点:(" << it->x() << ", " << it->y() << ", " << it->z() << ") " << std::endl;
    //         }
    //     }
    //     std::cout << std::endl;
    // }

    if (!errors.empty())
    {
        std::cerr << "导入过程中有错误:" << std::endl;
        for (const auto &err : errors)
        {
            std::cerr << err << std::endl;
        }
    }
    else
    {
        std::cout << "地图导入成功,lanelet数量: " << map->laneletLayer.size() << std::endl;
    }

    
    // 2. 创建交通规则对象
    auto trafficRules = lanelet::traffic_rules::TrafficRulesFactory::create(
        lanelet::Locations::Germany, lanelet::Participants::Vehicle);

    // 3. 创建RoutingCost
    lanelet::routing::RoutingCostPtrs costs{
        lanelet::routing::RoutingCostPtr(new lanelet::routing::RoutingCostDistance(0))
    };

    // 4. RoutingGraph配置
    const lanelet::routing::RoutingGraph::Configuration config;

    // 5. 调用build函数（注意参数类型）
    lanelet::routing::RoutingGraphUPtr routingGraph = lanelet::routing::RoutingGraph::build(
        *map, *trafficRules, costs, config);


    std::cout << "RoutingGraph 拓扑关系（每个lanelet的前驱和后继）:" << std::endl;
    for (const auto& llt : lanelets) {
        std::cout << "Lanelet ID: " << llt.id() << std::endl;

        // 打印前驱
        auto previous = routingGraph->previous(llt);
        std::cout << "  前驱: ";
        for (const auto& pre : previous) {
            std::cout << pre.id() << " ";
        }
        std::cout << std::endl;

        // 打印后继
        auto following = routingGraph->following(llt);
        std::cout << "  后继: ";
        for (const auto& fol : following) {
            std::cout << fol.id() << " ";
        }
        std::cout << std::endl;
    }

    // // 获取起点和终点点
    // const auto& start_pt = points[273];
    // const auto& end_pt = points[414];

    // 找到包含起点和终点的lanelet
    lanelet::Lanelet start_lanelet, end_lanelet;
    for (const auto& llt : lanelets) {
        if (lanelet::geometry::inside(llt, lanelet::BasicPoint2d(start_pt.x(), start_pt.y()))) {
            start_lanelet = llt;
        }
        if (lanelet::geometry::inside(llt, lanelet::BasicPoint2d(end_pt.x(), end_pt.y()))) {
            end_lanelet = llt;
        }
    }

    // 检查是否找到
    if (start_lanelet.id() == 0 || end_lanelet.id() == 0) {
        std::cerr << "未找到起点或终点所在的lanelet!" << std::endl;
        return 1;
    }
    else
    {
        std::cout << "起点所在的lanelet ID: " << start_lanelet.id() << ",长度"<< lanelet::geometry::length2d(start_lanelet) <<  std::endl;
        std::cout << "start_pt x: " << start_pt.x() << ",y:"<< start_pt.y() <<  std::endl;
        std::cout << "终点所在的lanelet ID: " << end_lanelet.id() << ",长度: " << lanelet::geometry::length2d(end_lanelet) << std::endl;
        std::cout << "end_pt x: " << end_pt.x() << ",y:"<< end_pt.y() <<  std::endl;
    }
    auto t_start = std::chrono::steady_clock::now();
    auto routeOpt = routingGraph->getRoute(start_lanelet, end_lanelet);
    if (!routeOpt) {
        std::cerr << "未找到全局路径!" << std::endl;
        return 1;
    }
    auto t_end = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed = t_end - t_start;
    std::cout << "程序运行时间: " << elapsed.count() << " 秒" << std::endl;
    const lanelet::routing::Route& route = *routeOpt;

    // 获取lanelet序列
    auto shortestPath = route.shortestPath();
    std::cout << "全局路径包含 " << shortestPath.size() << " 个lanelet:" << std::endl;
    for (const auto& llt : shortestPath) {
        std::cout << "Lanelet ID: " << llt.id() << std::endl;
    }

    // 1. 收集全局路径上的所有中心线点
    std::vector<lanelet::BasicPoint2d> center_points;
    for (const auto& llt : shortestPath) {
        std::cout << "shortestPath-lld-ID: " << llt.id() << ", Points: " << std::endl;
        for (const auto& pt : llt.centerline()) {
            center_points.emplace_back(pt.x(), pt.y());
            std::cout << "Point: (" << pt.x() << ", " << pt.y() << ")" << std::endl;
        }
    }

    // 2. 去除重复点（相邻lanelet可能有重复点）
    auto last = std::unique(center_points.begin(), center_points.end(), [](const auto& a, const auto& b){
        return (a.x() == b.x() && a.y() == b.y());
    });
    center_points.erase(last, center_points.end());

    // 3. 按0.5米间隔采样
    std::vector<lanelet::BasicPoint2d> sampled_points;
    if (!center_points.empty()) {
        sampled_points.push_back(center_points.front());
        double interval = 1;
        double remain = 0.0;
        for (size_t i = 1; i < center_points.size(); ++i) {
            lanelet::BasicPoint2d prev = center_points[i-1];
            lanelet::BasicPoint2d curr = center_points[i];
            double dx = curr.x() - prev.x();
            double dy = curr.y() - prev.y();
            double seg_len = std::hypot(dx, dy);
            double dist = remain;
            while (dist + interval <= seg_len) {
                double ratio = (dist + interval) / seg_len;
                double x = prev.x() + ratio * dx;
                double y = prev.y() + ratio * dy;
                sampled_points.emplace_back(x, y);
                dist += interval;
            }
            remain = seg_len - dist;
        }
    }

    // 4. 打印采样点
    std::cout << "全局路径采样点（间隔0.5米）数量: " << sampled_points.size() << std::endl;
    for (const auto& pt : sampled_points) {
        std::cout << "采样点: (" << pt.x() << ", " << pt.y() << ")" << std::endl;
    }

    return 0;
}

// #include <lanelet2_io/Io.h>
// #include <lanelet2_projection/LocalCartesian.h>
// #include <lanelet2_core/LaneletMap.h>
// #include <lanelet2_routing/RoutingGraph.h>
// #include <lanelet2_traffic_rules/TrafficRulesFactory.h>
// #include <lanelet2_routing/RoutingCost.h>
// #include <iostream>

// int main() {
//     // 1. 加载osm地图
//     std::string osm_file = "/home/byd-zpy/lanelet2_ws/src/lanelet2_test/map_file/new_lanelet2_maps_0715.osm";
//     lanelet::GPSPoint origin_gps{22.6881902, 114.3480115, 0.0};
//     lanelet::Origin origin(origin_gps);
//     lanelet::projection::LocalCartesianProjector projector(origin);
//     lanelet::ErrorMessages errors;
//     lanelet::LaneletMapPtr map = lanelet::load(osm_file, projector, &errors);

//     if (!errors.empty()) {
//         std::cerr << "导入过程中有错误:" << std::endl;
//         for (const auto &err : errors) {
//             std::cerr << err << std::endl;
//         }
//         return 1;
//     }

//     // 2. 创建交通规则对象
//     auto trafficRules = lanelet::traffic_rules::TrafficRulesFactory::create(
//         lanelet::Locations::Germany, lanelet::Participants::Vehicle);

//     // 3. 创建RoutingCost
//     lanelet::routing::RoutingCostPtrs costs{
//         lanelet::routing::RoutingCostPtr(new lanelet::routing::RoutingCostDistance(0))
//     };

//     // 4. RoutingGraph配置
//     const lanelet::routing::RoutingGraph::Configuration config;

//     // 5. 调用build函数（注意参数类型）
//     lanelet::routing::RoutingGraphUPtr routingGraph = lanelet::routing::RoutingGraph::build(
//         *map, *trafficRules, costs, config);

//     std::cout << "RoutingGraph 构建成功！" << std::endl;
//     return 0;
// }