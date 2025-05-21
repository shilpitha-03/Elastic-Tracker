#pragma once

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <Eigen/Core>
#include <vector>
#include <cstdint>

namespace mapping {

template <typename _Datatype>
struct RingBuffer {
 public:
  int size_x, size_y, size_z;
  std::vector<_Datatype> data;

  inline void setup(int _size_x, int _size_y, int _size_z) {
    size_x = _size_x;
    size_y = _size_y;
    size_z = _size_z;
    data.resize(size_x * size_y * size_z);
  }
  inline const int idx2add(int x, int N) const {
    return (x & N) >= 0 ? (x & N) : (x & N) + N;
  }
  inline const Eigen::Vector3i idx2add(const Eigen::Vector3i& id) const {
    return Eigen::Vector3i(
      idx2add(id.x(), size_x - 1),
      idx2add(id.y(), size_y - 1),
      idx2add(id.z(), size_z - 1)
    );
  }
  inline const _Datatype& at(const Eigen::Vector3i& ad) const {
    return data[(ad.z() * size_y + ad.y()) * size_x + ad.x()];
  }
  inline _Datatype& at(const Eigen::Vector3i& ad) {
    return data[(ad.z() * size_y + ad.y()) * size_x + ad.x()];
  }
  inline _Datatype* atPtr(const Eigen::Vector3i& ad) {
    return &data[(ad.z() * size_y + ad.y()) * size_x + ad.x()];
  }
  inline const _Datatype& atId(const Eigen::Vector3i& id) const {
    return at(idx2add(id));
  }
  inline _Datatype& atId(const Eigen::Vector3i& id) {
    return at(idx2add(id));
  }
  inline _Datatype* atIdPtr(const Eigen::Vector3i& id) {
    return atPtr(idx2add(id));
  }
  inline void fillData(const _Datatype& val) {
    std::fill(data.begin(), data.end(), val);
  }
};

struct OccGridMap {
 public:
  int p_min, p_max, p_hit, p_mis, p_occ, p_def;
  int inflate_size;
  double sensor_range;

  bool init_finished = false;
  int offset_x, offset_y, offset_z;

  double resolution;
  int size_x, size_y, size_z;

 private:
  RingBuffer<int8_t>  infocc;
  RingBuffer<int8_t>  vis;
  RingBuffer<int16_t> pro;
  RingBuffer<uint16_t> occ;

 public:
  inline void setup(const double& res,
                    const Eigen::Vector3d& map_size,
                    const double& cam_range,
                    bool use_global_map = false) {
    resolution = res;
    size_x = std::exp2(int(std::log2(map_size.x()/res)));
    size_y = std::exp2(int(std::log2(map_size.y()/res)));
    size_z = std::exp2(int(std::log2(map_size.z()/res)));
    if (use_global_map) {
      offset_x = -size_x; offset_y = -size_y; offset_z = -size_z;
      size_x*=2; size_y*=2; size_z*=2;
    }
    infocc.setup(size_x,size_y,size_z);
    occ.setup(size_x,size_y,size_z);
    vis.setup(size_x,size_y,size_z);
    occ.fillData(0);
    infocc.fillData(0);
    if (use_global_map) return;
    pro.setup(size_x,size_y,size_z);
    pro.fillData(p_def);
    sensor_range = cam_range;
  }

  inline void setupP(const int& _p_min,
                     const int& _p_max,
                     const int& _p_hit,
                     const int& _p_mis,
                     const int& _p_occ,
                     const int& _p_def) {
    p_min = _p_min; p_max = _p_max; p_hit = _p_hit;
    p_mis = _p_mis; p_occ = _p_occ; p_def = _p_def;
  }

  inline Eigen::Vector3i pos2idx(const Eigen::Vector3d& pt) const {
    return (pt / resolution).array().floor().cast<int>();
  }
  inline Eigen::Vector3d idx2pos(const Eigen::Vector3i& id) const {
    return (id.cast<double>() + Eigen::Vector3d(0.5,0.5,0.5)) * resolution;
  }

  inline bool isInMap(const Eigen::Vector3i& id) const {
    return !((id.x()-offset_x)&(~(size_x-1)))
        && !((id.y()-offset_y)&(~(size_y-1)))
        && !((id.z()-offset_z)&(~(size_z-1)));
  }
  inline bool isInMap(const Eigen::Vector3d& p) const {
    return isInMap(pos2idx(p));
  }

  inline bool isInSmallMap(const Eigen::Vector3i& id) const {
    return id.x()-offset_x >= inflate_size
        && id.x()-offset_x < size_x - inflate_size
        && id.y()-offset_y >= inflate_size
        && id.y()-offset_y < size_y - inflate_size
        && id.z()-offset_z >= inflate_size
        && id.z()-offset_z < size_z - inflate_size;
  }
  inline bool isInSmallMap(const Eigen::Vector3d& p) const {
    return isInSmallMap(pos2idx(p));
  }

  template <typename _Msgtype>
  inline void to_msg(_Msgtype& msg) {
    msg.resolution = resolution;
    msg.size_x = size_x; msg.size_y = size_y; msg.size_z = size_z;
    msg.offset_x = offset_x; msg.offset_y = offset_y; msg.offset_z = offset_z;
    msg.data = infocc.data;
  }

  template <typename _Msgtype>
  inline void from_msg(const _Msgtype& msg) {
    resolution = msg.resolution;
    size_x = msg.size_x; size_y = msg.size_y; size_z = msg.size_z;
    offset_x = msg.offset_x; offset_y = msg.offset_y; offset_z = msg.offset_z;
    infocc.setup(size_x,size_y,size_z);
    infocc.data = msg.data;
  }

  inline void setOcc(const Eigen::Vector3d& p) {
    infocc.atId(pos2idx(p)) = 1;
  }

  inline bool isOccupied(const Eigen::Vector3i& id) const {
    return isInMap(id) && infocc.atId(id)==1;
  }
  inline bool isOccupied(const Eigen::Vector3d& p) const {
    return isOccupied(pos2idx(p));
  }
  inline bool isUnKnown(const Eigen::Vector3i& id) const {
    return !isInMap(id) || infocc.atId(id)==0;
  }
  inline bool isUnKnown(const Eigen::Vector3d& p) const {
    return isUnKnown(pos2idx(p));
  }
  inline void setFree(const Eigen::Vector3i& id) {
    if(isInMap(id)) infocc.atId(id) = -1;
  }
  inline void setFree(const Eigen::Vector3d& p) {
    setFree(pos2idx(p));
  }
  inline void setFree(const Eigen::Vector3d& ld, const Eigen::Vector3d& ru) {
    Eigen::Vector3i id_ld=pos2idx(ld), id_ru=pos2idx(ru);
    for(int x=id_ld.x(); x<=id_ru.x(); ++x)
      for(int y=id_ld.y(); y<=id_ru.y(); ++y)
        for(int z=id_ld.z(); z<=id_ru.z(); ++z)
          setFree(Eigen::Vector3i{x,y,z});
  }

  void updateMap(const Eigen::Vector3d& sensor_p,
                 const std::vector<Eigen::Vector3d>& pc);
  void occ2pc(sensor_msgs::msg::PointCloud2& msg);
  void occ2pc(sensor_msgs::msg::PointCloud2& msg, double floor, double ceil);
  void inflate_once();
  void inflate_xy();
  void inflate_last();
  void inflate(int inflate_size);
};

}  // namespace mapping
