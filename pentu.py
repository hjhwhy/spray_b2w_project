from pyproj import Transformer

# 输入 EPSG:4326（纬度/经度）
# 输出 EPSG:2100（Greek Grid 投影）
transformer = Transformer.from_crs("epsg:4326", "epsg:2100", always_xy=True)

lon, lat = 23.79313, 38.04166
x, y = transformer.transform(lon, lat)


print("经度:", lon)
print("纬度:", lat)

print("Easting:", x)
print("Northing:", y)

