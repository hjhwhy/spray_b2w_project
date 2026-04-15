//拿希腊经纬度坐标做转换，验证投影的误差

#include <iostream>
#include <proj.h>
#include <iomanip>

int main() {
    PJ_CONTEXT* ctx = proj_context_create();
    // 定义源坐标系 (WGS84, EPSG:4326)
    PJ* pj_wgs84 = proj_create(ctx, "EPSG:4326");
    // 目标坐标系 (EPSG:2100)
    PJ* pj_2100 = proj_create(ctx, "EPSG:2100");
    PJ* pj_transform = proj_create_crs_to_crs(ctx, "EPSG:4326", "EPSG:2100", nullptr);
    if (!pj_transform) {
        std::cerr << "Failed to create transformation." << std::endl;
        proj_context_destroy(ctx);
        return 1;
    }
    double lon = 23.7275;  // 雅典的经度
    double lat = 37.9838;  // 雅典的纬度
    // PROJ 使用 PJ_COORD 结构
    PJ_COORD input = proj_coord(lat, lon, 0, 0);  // 注意：proj_coord(lat, lon, z, t)
    
    PJ_COORD output = proj_trans(pj_transform, PJ_FWD, input);
    if (proj_errno(pj_transform)) {
        std::cerr << "Transformation error: " << proj_errno_string(proj_errno(pj_transform)) << std::endl;
        proj_destroy(pj_transform);
        proj_context_destroy(ctx);
        return 1;
    }
    std::cout << std::fixed << std::setprecision(3) << "EPSG:2100 坐标 -> Easting: " << output.xy.x
          << ", Northing: " << output.xy.y << " (米)" << std::endl;
    proj_destroy(pj_transform);
    proj_destroy(pj_wgs84);
    proj_destroy(pj_2100);
    proj_context_destroy(ctx);
    return 0;
}