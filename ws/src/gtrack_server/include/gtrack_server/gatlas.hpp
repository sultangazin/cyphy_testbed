#ifndef GATLAS_HPP
#define GATLAS_HPP


#include <unordered_map>
#include <set>
#include <Eigen/Dense>

struct TransformData {
    Eigen::Quaterniond rot;
    Eigen::Vector3d t;

    TransformData() : 
        rot(Eigen::Quaterniond::Identity()),
        t(Eigen::Vector3d::Zero()) {}
};

struct AtlasElement {
    int id;
    std::unordered_map<int, TransformData> relations; 
};

class GAtlas {
    public: 
        GAtlas();
        ~GAtlas();

        void insert(int a, int b, const TransformData& TR);
        bool getTransform(int id_origin, int id_dest, TransformData& tr);

    private:
        std::unordered_map<int, AtlasElement> _gatlas;
        std::set<int> _visited;

        bool find_path(int s, int e, TransformData& TR);
};

#endif
