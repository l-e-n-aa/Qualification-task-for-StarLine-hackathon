// Стандартные библиотеки C++
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <cmath>
#include <random>
#include <algorithm>
#include <limits>

// Библиотеки для работы с математикой и линейной алгеброй
#include <Eigen/Dense>
#include <Eigen/Geometry>

// ROS 2 основные заголовки
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

// ROS 2 TF2
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/create_timer_ros.h>  
#include <tf2_ros/transform_listener.h>  
#include <tf2/exceptions.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>


// Итераторы для PointCloud2
#include <sensor_msgs/point_cloud2_iterator.hpp>

// Для визуализации плоскостей
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using namespace std::chrono_literals;


// Явная специализация для линковки
template void tf2::doTransform<sensor_msgs::msg::PointCloud2>(
    const sensor_msgs::msg::PointCloud2&, 
    sensor_msgs::msg::PointCloud2&, 
    const geometry_msgs::msg::TransformStamped&);

class ICP;
class Filter_cloud;
class ICP_node;
class Cloud;


class Cloud {
public:
    Cloud() {}
    
    Cloud(const std::vector<Eigen::Vector3d>& cloud_points) 
        : cloud_points(cloud_points) {
        multi_find_planes();

        for (Eigen::Vector3d p : cloud_points) cloud_centroid += p;
        cloud_centroid /= cloud_points.size();
    }
    
    struct Plane {
        Eigen::Vector4d coefficients;
        Eigen::Vector3d normal;
        std::vector<size_t> point_indices;
        Eigen::Vector3d centroid;

        bool operator==(const Plane& other) const {
            return normal.isApprox(other.normal) && 
                   std::abs(coefficients[3] - other.coefficients[3]) < 1e-10;
        }
        
        Plane() = default;
        Plane(const Eigen::Vector4d& coeffs, const Eigen::Vector3d& norm, 
              const std::vector<size_t>& indices)
            : coefficients(coeffs), normal(norm), point_indices(indices) {}
    };

    std::vector<Eigen::Vector3d> get_points() {
        return cloud_points;
    }
    
    std::vector<Cloud::Plane> get_planes() const {
        return planes;
    }

    // Метод для фильтрации точек пола
    std::vector<Eigen::Vector3d> get_points_without_floor(double floor_height_threshold = 0.1) const {
        std::vector<Eigen::Vector3d> filtered_points;
        filtered_points.reserve(cloud_points.size());
        
        for (const auto& point : cloud_points) {
            // Исключаем точки близкие к полу (вертикальные плоскости и маркеры)
            if (std::abs(point.z()) > floor_height_threshold) {
                filtered_points.push_back(point);
            }
        }
        
        return filtered_points;
    }

private:
    std::vector<Eigen::Vector3d> cloud_points;
    std::vector<Plane> planes;
    Eigen::Vector3d cloud_centroid; 

    void multi_find_planes() {
        if (cloud_points.empty()) return;

        std::vector<size_t> indices(cloud_points.size());
        for (size_t i = 0; i < cloud_points.size(); i++) {
            indices[i] = i;
        }
        int counter = 0;
        // Ищем основные плоскости
        for (int i = 0; i < 3 && indices.size() > cloud_points.size() * 0.05; i++) {
            Plane plane = simple_plane_detect(indices, 100);
            RCLCPP_DEBUG(rclcpp::get_logger("Cloud"), "\n\t\tFound plane: normal=(%.3f,%.3f,%.3f), points=%zu, d=%.3f",plane.normal.x(), plane.normal.y(), plane.normal.z(),plane.point_indices.size(), plane.coefficients[3]);

            counter++;
            if(counter >= 1e2) break;

            // Простая классификация
            if (std::abs(plane.normal.z() - 1) <= 0.1) {
                // Горизонтальная плоскость
                plane.normal = Eigen::Vector3d(0, 0, 1);
                if (abs(plane.coefficients[3]) >= 0.1) {
                    i--;
                    continue;
                }
            } else {
                // Вертикальная плоскость
                plane.normal.z() = 0;
                plane.normal.normalize();
            }

            if (plane.point_indices.size() <= 150) {
                i--;
                continue;
            }


            // Удаляем найденные точки
            std::vector<size_t> new_indices;
            for (size_t idx : indices) {
                if (std::find(plane.point_indices.begin(), plane.point_indices.end(), idx) == plane.point_indices.end()) {
                    new_indices.push_back(idx);
                }
            }
            indices = new_indices;
            
            // Пересчитываем коэффициенты
            Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
            for (size_t idx : plane.point_indices) {
                centroid += cloud_points[idx];
            }
            centroid /= plane.point_indices.size();
            plane.coefficients << plane.normal, -plane.normal.dot(centroid);
            if(plane.coefficients[3] < 0) {
                plane.coefficients *= -1;
                plane.normal *= -1;
            }
            plane.centroid = centroid;
            if (std::abs(std::abs(plane.normal.z()) - 1) <= 0.1) {
                // Горизонтальная плоскость
                if (abs(plane.coefficients[3]) >= 0.1) {
                    i--;
                    continue;
                }
            }
            planes.push_back(plane);
        }
    }

    Plane simple_plane_detect(const std::vector<size_t>& indices, int max_iterations) {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<size_t> dis(0, indices.size() - 1);
        
        Plane best_plane;
        size_t best_inliers = 0;

        for (int i = 0; i < max_iterations; ++i) {
            // 3 случайные точки
            size_t idx1 = dis(gen), idx2 = dis(gen), idx3 = dis(gen);
            if (idx1 == idx2 || idx1 == idx3 || idx2 == idx3) {
                i--;
                continue;
            }
            const auto& p1 = cloud_points[indices[idx1]];
            const auto& p2 = cloud_points[indices[idx2]];
            const auto& p3 = cloud_points[indices[idx3]];

            // Нормаль плоскости
            Eigen::Vector3d normal = (p2 - p1).cross(p3 - p1).normalized();
            if (normal.norm() < 1e-6) continue;

            if (std::abs(std::abs(normal.z()) - 1) <= 1e-4) {
                // Горизонтальная плоскость
                normal = Eigen::Vector3d(0, 0, 1);
            } else if (std::abs(normal.z()) <= 1e-4) {
                // Вертикальная плоскость
                normal.z() = 0;
            }
            else {
                i--;
                continue;
            }
            normal.normalize();

            double D = -normal.dot(p1);
            if (std::abs(std::abs(normal.z()) - 1) <= 0.1) if (std::abs(D) >= 0.06) continue;
            

            // Считаем inliers
            std::vector<size_t> inliers;
            for (size_t j = 0; j < indices.size(); ++j) {
                const auto& p = cloud_points[indices[j]];
                if (std::abs(normal.dot(p) + D) < 0.04) {
                    inliers.push_back(indices[j]);
                }
            }

            if (inliers.size() > best_inliers) {
                best_inliers = inliers.size();
                Eigen::Vector4d coeffs;
                coeffs << normal, D;
                best_plane = Plane{coeffs, normal, inliers};
            }
        }
        return best_plane;
    }
};

class ICP {
public:
    ICP() : processing_(false), stop_requested_(false) {}

    sensor_msgs::msg::PointCloud2 get_wall_PC(){
        std::lock_guard<std::mutex> lock(wall_mutex_);
        return wall_result;
    }
    
    sensor_msgs::msg::PointCloud2 get_marker_PC(){
        return marker_result;
    }

    void add_wall_PC(const sensor_msgs::msg::PointCloud2& PC) {
        std::lock_guard<std::mutex> lock(wall_mutex_);
        wall_pointclouds.emplace_back(PC);
    }

    void add_marker_PC(const sensor_msgs::msg::PointCloud2& PC) {
        std::lock_guard<std::mutex> lock(marker_mutex_);
        marker_pointclouds.emplace_back(PC);
    }

    void start_processing(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr wall_pub, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr marker_pub, rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr planes_pub) {
        this->wall_pub = wall_pub;
        this->marker_pub = marker_pub;
        this->planes_pub = planes_pub;
        if (processing_.exchange(true)) {
            return; // Уже работает
        }
        
        processing_thread_ = std::thread([this]() {
            process_loop();
        });
    }

    void stop_processing() {
        stop_requested_ = true;
        if (processing_thread_.joinable()) {
            processing_thread_.join();
        }
        processing_ = false;
    }

    bool is_processing() const { return processing_; }

private:
    std::vector<sensor_msgs::msg::PointCloud2> wall_pointclouds;
    std::vector<sensor_msgs::msg::PointCloud2> marker_pointclouds;
    sensor_msgs::msg::PointCloud2 wall_result, marker_result;
    std::mutex wall_mutex_, marker_mutex_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr wall_pub, marker_pub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr planes_pub;

    std::thread processing_thread_;
    std::atomic<bool> processing_;
    std::atomic<bool> stop_requested_;

    // Узел KD-дерева
    struct KDNode {
        Eigen::Vector3d point;
        int axis; // 0=x, 1=y, 2=z
        std::unique_ptr<KDNode> left;
        std::unique_ptr<KDNode> right;

        KDNode(const Eigen::Vector3d& p, int a) 
            : point(p), axis(a), left(nullptr), right(nullptr) {}
    };

    // Рекурсивное построение KD-дерева
    std::unique_ptr<KDNode> build_kdtree(std::vector<Eigen::Vector3d>& points, int depth = 0) {
        if (points.empty()) return nullptr;

        int axis = depth % 3;
        size_t median = points.size() / 2;

        // Сортировка по текущей оси
        std::nth_element(
            points.begin(), points.begin() + median, points.end(),
            [axis](const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
                return a[axis] < b[axis];
            }
        );

        // Создание узла
        auto node = std::make_unique<KDNode>(points[median], axis);
        
        // Рекурсивное построение поддеревьев
        std::vector<Eigen::Vector3d> left_points(points.begin(), points.begin() + median);
        std::vector<Eigen::Vector3d> right_points(points.begin() + median + 1, points.end());
        
        node->left = build_kdtree(left_points, depth + 1);
        node->right = build_kdtree(right_points, depth + 1);

        return node;
    }

    // Поиск ближайшей точки в KD-дереве
    void find_nearest(const KDNode* node, const Eigen::Vector3d& query, Eigen::Vector3d& best_point, double& best_dist) const {
        if (!node) return;

        // Проверяем текущую точку
        double dist = (query - node->point).squaredNorm();
        if (dist < best_dist) {
            best_dist = dist;
            best_point = node->point;
        }

        // Определяем направление поиска
        double axis_diff = query[node->axis] - node->point[node->axis];
        bool go_left = axis_diff <= 0;

        // Рекурсивный поиск в нужном поддереве
        find_nearest(go_left ? node->left.get() : node->right.get(), 
                    query, best_point, best_dist);

        // Проверяем, нужно ли искать в другом поддереве
        if (axis_diff * axis_diff < best_dist) {
            find_nearest(go_left ? node->right.get() : node->left.get(), 
                        query, best_point, best_dist);
        }
    }

    // Поиск пар точек
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> find_closest_point_pairs(const KDNode* target, const std::vector<Eigen::Vector3d>& source, double max_distance) {
        
        std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> pairs;
        pairs.reserve(source.size());
        
        for(const auto& point : source){
            Eigen::Vector3d best_point = Eigen::Vector3d::Zero();
            double best_dist = std::numeric_limits<double>::max();
            
            find_nearest(target, point, best_point, best_dist);
            
            if(best_dist <= max_distance * max_distance) {
                pairs.emplace_back(best_point, point);
            }
        }
        
        return pairs;
    }

    std::vector<Eigen::Vector3d> convert_PC_to_vector(const sensor_msgs::msg::PointCloud2& PC) {
        std::vector<Eigen::Vector3d> vector;
        vector.reserve(PC.width * PC.height);
        
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(PC, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(PC, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(PC, "z");
        
        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
            if (std::isfinite(iter_x[0]) && std::isfinite(iter_y[0]) && std::isfinite(iter_z[0])) {
                vector.emplace_back(iter_x[0], iter_y[0], iter_z[0]);
            }
        }
        
        return vector;
    }

    sensor_msgs::msg::PointCloud2 convert_vector_to_PC(const std::vector<Eigen::Vector3d>& vector, const rclcpp::Time& stamp) {
        sensor_msgs::msg::PointCloud2 cloud;
    
        // Настройка заголовка
        cloud.header.stamp = stamp;
        cloud.header.frame_id = "map";
        
        // Настройка полей
        cloud.height = 1;
        cloud.width = vector.size();
        cloud.is_bigendian = false;
        cloud.is_dense = true;
        
        // Определение полей (x, y, z)
        sensor_msgs::PointCloud2Modifier modifier(cloud);
        modifier.setPointCloud2Fields(3,
            "x", 1, sensor_msgs::msg::PointField::FLOAT32,
            "y", 1, sensor_msgs::msg::PointField::FLOAT32,
            "z", 1, sensor_msgs::msg::PointField::FLOAT32);
        
        // Подготовка данных
        modifier.resize(vector.size());
        
        // Заполнение через итератор
        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
        
        for (size_t i = 0; i < vector.size(); ++i, ++iter_x, ++iter_y, ++iter_z) {
            *iter_x = static_cast<float>(vector[i].x());
            *iter_y = static_cast<float>(vector[i].y());
            *iter_z = static_cast<float>(vector[i].z());
        }
        
        return cloud;
    }

    void compute_transform(const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>& pairs, Eigen::Matrix3d& R, Eigen::Vector3d& t) {
        if (pairs.size() < 3) {
            R = Eigen::Matrix3d::Identity();
            t = Eigen::Vector3d::Zero();
            return;
        }

        // Вычисление центроидов
        Eigen::Vector3d src_centroid = Eigen::Vector3d::Zero();
        Eigen::Vector3d tgt_centroid = Eigen::Vector3d::Zero();
        
        for (const auto& [tgt, src] : pairs) {
            src_centroid += src;
            tgt_centroid += tgt;
        }
        
        src_centroid /= pairs.size();
        tgt_centroid /= pairs.size();

        // Ковариационная матрица
        Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
        for (const auto& [tgt, src] : pairs) {
            H += (src - src_centroid) * ((tgt - tgt_centroid).transpose());
        }

        // SVD разложение
        Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
        R = svd.matrixV() * svd.matrixU().transpose();
        
        // Коррекция отражений
        if (R.determinant() < 0) {
            Eigen::Matrix3d V = svd.matrixV();
            V.col(2) *= -1;
            R = V * svd.matrixU().transpose();
        }
        
        t = tgt_centroid - R * src_centroid;
    }

    void concatenate_pointclouds(sensor_msgs::msg::PointCloud2& result_cloud, const sensor_msgs::msg::PointCloud2& cloud2) {
        if (result_cloud.point_step != cloud2.point_step || result_cloud.fields.size() != cloud2.fields.size()) {
            return;
        }

        size_t original_size = result_cloud.data.size();
        result_cloud.data.resize(original_size + cloud2.data.size());
        memcpy(&result_cloud.data[original_size], &cloud2.data[0], cloud2.data.size());

        result_cloud.width += cloud2.width;
        result_cloud.row_step = result_cloud.width * result_cloud.point_step;
        
        if (rclcpp::Time(cloud2.header.stamp) > rclcpp::Time(result_cloud.header.stamp)) {
            result_cloud.header.stamp = cloud2.header.stamp;
        }
    }

    sensor_msgs::msg::PointCloud2 transform_PointCloud(const sensor_msgs::msg::PointCloud2& input_cloud, const tf2::Transform& tf) {
        sensor_msgs::msg::PointCloud2 output_cloud = input_cloud;
        
        sensor_msgs::PointCloud2Iterator<float> iter_x(output_cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(output_cloud, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(output_cloud, "z");

        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
            // Создаем tf2 вектор из координат точки
            tf2::Vector3 point(*iter_x, *iter_y, *iter_z);
            
            // Применяем полное преобразование (вращение + трансляция)
            tf2::Vector3 transformed_point = tf * point;
            
            // Записываем результат
            *iter_x = static_cast<float>(transformed_point.x());
            *iter_y = static_cast<float>(transformed_point.y());
            *iter_z = static_cast<float>(transformed_point.z());
        }
        
        return output_cloud;
    }

    void iteration_process(const KDNode* target, std::vector<Eigen::Vector3d>& source, Eigen::Matrix3d& R_result, Eigen::Vector3d& t_result, int iterations, double max_dist) {
        for(int i = 0; i < iterations; i++){
            auto pairs = find_closest_point_pairs(target, source, max_dist);
            
            if (pairs.size() < 3) {
                RCLCPP_WARN(rclcpp::get_logger("ICP"), "Too few correspondences: %zu", pairs.size());
                break;
            }
            
            Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
            Eigen::Vector3d t = Eigen::Vector3d::Zero();
            compute_transform(pairs, R, t);
            
            // Обновляем общую трансформацию
            R_result = R * R_result;
            t_result = R * t_result + t;
            
            // Применяем трансформацию к точкам
            for (Eigen::Vector3d& p : source) {
                p = R * p + t;
            }
        }
    }

    void run_ICP_Iterations(const KDNode* target, const std::vector<Eigen::Vector3d>& s, tf2::Transform& tf_out, int iterations) {
        std::vector<Eigen::Vector3d> source = s;
        Eigen::Matrix3d R_result = Eigen::Matrix3d::Identity();
        Eigen::Vector3d t_result = Eigen::Vector3d::Zero();

        // Грубое приближение
        iteration_process(target, source, R_result, t_result, iterations / 3, 0.5);
        // Точное приближение
        iteration_process(target, source, R_result, t_result, iterations - iterations / 3, 0.05);
        
        tf2::Vector3 transform(t_result.x(), t_result.y(), t_result.z());
        Eigen::Quaterniond q(R_result);
        q.normalize();
        
        if (!q.coeffs().allFinite()) {
            q = Eigen::Quaterniond::Identity();
        }
        
        tf2::Quaternion quat(q.x(), q.y(), q.z(), q.w());
        
        tf_out.setOrigin(transform);
        tf_out.setRotation(quat);
    }

    tf2::Transform compute_simple_transform(const Cloud::Plane& target, const Cloud::Plane& source) {
        tf2::Transform result = tf2::Transform::getIdentity();
        
        // Для параллельных плоскостей - только трансляция вдоль нормали
        if (target.normal.dot(source.normal) > 0.95) {
            double translation = target.coefficients[3] - source.coefficients[3];
            Eigen::Vector3d t = translation * target.normal;
            result.setOrigin(tf2::Vector3(t.x(), t.y(), t.z()));
        }
        
        return result;
    }
    
    tf2::Transform compute_transform_between_planes(const Cloud::Plane& target, const Cloud::Plane& source) {
        tf2::Transform result = tf2::Transform::getIdentity();
        
        // Вычисляем вращение между нормалями
        Eigen::Vector3d v1 = target.normal.normalized();
        Eigen::Vector3d v2 = source.normal.normalized();
        
        // Убеждаемся, что нормали смотрят в одном направлении
        if (v1.dot(v2) < 0) {
            v2 = -v2;
        }
        
        Eigen::Quaterniond q;
        q.setFromTwoVectors(v2, v1);
        q.normalize();
        
        // Вращаем исходный центроид
        Eigen::Vector3d source_centroid_rotated = q * source.centroid;

        // Пересчитываем коэффициенты плоскости
        double d = -v1.dot(source_centroid_rotated);
        double d_diff = target.coefficients[3] - d;

        // Полная трансляция
        Eigen::Vector3d translation = -v1 * d_diff;
        
        result.setOrigin(tf2::Vector3(translation.x(), translation.y(), translation.z()));
        result.setRotation(tf2::Quaternion(q.x(), q.y(), q.z(), q.w()));
        
        return result;
    }

    std::vector<std::pair<Cloud::Plane, Cloud::Plane>> improved_plane_matching(const std::vector<Cloud::Plane>& target_planes, const std::vector<Cloud::Plane>& source_planes) {
        
        std::vector<std::pair<Cloud::Plane, Cloud::Plane>> pairs;
        std::vector<bool> matched_source(source_planes.size(), false);
        
        // Сначала сопоставляем пол (горизонтальные плоскости)
        for (size_t t_idx = 0; t_idx < target_planes.size(); t_idx++) {
            const auto& t_plane = target_planes[t_idx];
            
            if (std::abs(t_plane.normal.z()) > 0.9) { // Это пол
                double best_score = -1.0;
                size_t best_s_idx = -1;
                
                for (size_t s_idx = 0; s_idx < source_planes.size(); s_idx++) {
                    if (matched_source[s_idx]) continue;
                    
                    const auto& s_plane = source_planes[s_idx];
                    if (std::abs(s_plane.normal.z()) < 0.9) continue; // Не пол
                    
                    // Скор учитывает схожесть нормалей и расстояние
                    double normal_similarity = t_plane.normal.dot(s_plane.normal);
                    double distance_similarity = 1.0 / (1.0 + std::abs(t_plane.coefficients[3] - s_plane.coefficients[3]));
                    double score = normal_similarity * distance_similarity;
                    
                    if (score > best_score && score > 0.7) {
                        best_score = score;
                        best_s_idx = s_idx;
                    }
                }
                
                if (best_s_idx != -1) {
                    matched_source[best_s_idx] = true;
                    pairs.emplace_back(t_plane, source_planes[best_s_idx]);
                }
            }
        }
        
        // Затем сопоставляем стены (вертикальные плоскости)
        for (size_t t_idx = 0; t_idx < target_planes.size(); t_idx++) {
            const auto& t_plane = target_planes[t_idx];
            
            if (std::abs(t_plane.normal.z()) < 0.1) { // Это стена
                double best_score = -1.0;
                size_t best_s_idx = -1;
                
                for (size_t s_idx = 0; s_idx < source_planes.size(); s_idx++) {
                    if (matched_source[s_idx]) continue;
                    
                    const auto& s_plane = source_planes[s_idx];
                    if (std::abs(s_plane.normal.z()) > 0.1) continue; // Не стена
                    
                    double normal_similarity = t_plane.normal.dot(s_plane.normal);
                    double dist_diff = std::abs(t_plane.coefficients[3] - s_plane.coefficients[3]) / 5;
                    double score = normal_similarity / dist_diff;
                    
                    if (score > best_score && score > 0.8 && dist_diff <= 0.6) {
                        best_score = score;
                        best_s_idx = s_idx;
                    }
                }
                
                if (best_s_idx != -1) {
                    matched_source[best_s_idx] = true;
                    pairs.emplace_back(t_plane, source_planes[best_s_idx]);
                }
            }
        }
        
        std::sort(pairs.begin(), pairs.end(), [](const std::pair<Cloud::Plane, Cloud::Plane>& a, const std::pair<Cloud::Plane, Cloud::Plane>& b) {
            double similarity_a = a.first.normal.dot(a.second.normal), similarity_b = b.first.normal.dot(b.second.normal);
            double dist_diff_a = std::abs(a.first.coefficients[3] - a.second.coefficients[3]) / 5, dist_diff_b = std::abs(b.first.coefficients[3] - b.second.coefficients[3]) / 5;
            similarity_a /= dist_diff_a;
            similarity_b /= dist_diff_b;
            return similarity_a > similarity_b;

            //return a.first.point_indices.size() > b.first.point_indices.size()z;
        });

        RCLCPP_DEBUG(rclcpp::get_logger("Cloud"), "Found pairs: %zu", pairs.size());
        for(auto& [plane, plane2] : pairs) {
            RCLCPP_DEBUG(rclcpp::get_logger("Cloud"), "\n\tFound plane: normal=(%.3f,%.3f,%.3f), points=%zu, d=%.3f",plane.normal.x(), plane.normal.y(), plane.normal.z(),plane.point_indices.size(), plane.coefficients[3]);
            RCLCPP_DEBUG(rclcpp::get_logger("Cloud"), "\n\tFound plane2: normal=(%.3f,%.3f,%.3f), points=%zu, d=%.3f",plane2.normal.x(), plane2.normal.y(), plane2.normal.z(),plane2.point_indices.size(), plane2.coefficients[3]);
        }
        
        return pairs;
    }

    double get_rotation_angle(const tf2::Quaternion& quat) {
        // Вычисляем угол вращения из кватерниона
        return 2.0 * std::acos(std::min(std::max(quat.w(), -1.0), 1.0));
    }

    tf2::Transform ICP_iterations_plane(Cloud &target_cloud, Cloud &source_cloud) {
        auto target_planes = target_cloud.get_planes();
        auto source_planes = source_cloud.get_planes();
        tf2::Transform result = tf2::Transform::getIdentity();

        // Публикуем найденные плоскости для визуализации
        publish_planes(target_cloud.get_planes(), "target_planes");
        publish_planes(source_cloud.get_planes(), "source_planes");

        if (target_planes.empty() || source_planes.empty()) {
            return tf2::Transform::getIdentity();
        }
        
        auto pairs = improved_plane_matching(target_planes, source_planes);
        if (pairs.empty()) {
            return tf2::Transform::getIdentity();
        }
        
        // Более реалистичные ограничения
        const double MAX_ROTATION_ANGLE = 0.5236; // 30
        const double MAX_TRANSLATION = 0.6; // 50 см
        
        // Взвешенное среднее трансформаций вместо умножения
        tf2::Vector3 combined_translation(0, 0, 0);
        tf2::Quaternion combined_rotation(0, 0, 0, 1);
        double total_weight = 0.0;

        for (size_t i = 0; i < pairs.size(); i++) {
            auto& pair = pairs[i];
            if (pair.first.normal.z() >= 0.9) {
                tf2::Transform local_transform = compute_simple_transform(pair.first, pair.second);
                result *= local_transform;
                pairs.erase(pairs.begin() + i);
                break;
            }
        }

        for (size_t i = 0; i < pairs.size(); i++) {
            auto& pair = pairs[i];
            tf2::Transform local_transform;
            
            if (std::abs(pair.first.normal.z()) > 0.9) { // Горизонтальная плоскость (пол)
                local_transform = compute_simple_transform(pair.first, pair.second);
            } else { // Вертикальная плоскость (стена)
                local_transform = compute_transform_between_planes(pair.first, pair.second);
            }
            
            // Проверяем величину вращения
            double angle = get_rotation_angle(local_transform.getRotation());
            
            if (std::abs(angle) > MAX_ROTATION_ANGLE) {
                RCLCPP_WARN(rclcpp::get_logger("ICP"), 
                    "Rotation angle %.3f rad (%.1f°) exceeds limit, clamping to %.3f rad",
                    angle, angle * 180/M_PI, MAX_ROTATION_ANGLE);
                continue;
                // Ограничиваем угол вращения
                tf2::Quaternion limited_rot = local_transform.getRotation();
                limited_rot.normalize();
                local_transform.setRotation(limited_rot);
            }
            
            // Проверяем величину трансляции
            double translation_length = local_transform.getOrigin().length();
            if (translation_length > MAX_TRANSLATION) {
                RCLCPP_WARN(rclcpp::get_logger("ICP"), 
                    "Translation length %.3f exceeds limit, clamping to %.3f",
                    translation_length, MAX_TRANSLATION);
                continue;
                // Ограничиваем трансляцию
                tf2::Vector3 limited_translation = local_transform.getOrigin().normalized() * MAX_TRANSLATION;
                local_transform.setOrigin(limited_translation);
            }
            
            //result = result * local_transform;

            // Взвешиваем по количеству точек в плоскости
            double weight = pair.first.point_indices.size();
            total_weight += weight;
            
            // Накопление взвешенной трансляции
            combined_translation += local_transform.getOrigin() * weight;
            
            // Накопление взвешенного вращения (используем slerp для интерполяции)
            if (combined_rotation.length2() < 1e-10) {
                combined_rotation = local_transform.getRotation();
            } else {
                combined_rotation = combined_rotation.slerp(local_transform.getRotation(), weight / (total_weight + weight));
            }
        }
        
        // Нормализуем результат
        if (total_weight > 0) {
            combined_translation = combined_translation / total_weight;
        }
        combined_rotation.normalize();
        
        tf2::Transform final_transform = result;
        final_transform.setOrigin(combined_translation);
        final_transform.setRotation(combined_rotation);
        
        RCLCPP_DEBUG(rclcpp::get_logger("ICP"), 
            "ICP debugging results:"
            "\nResult transform:"
            "\n\t translation: %.3f, %.3f, %.3f"
            "\n\t rotation: %.3f, %.3f, %.3f, %.3f",
            final_transform.getOrigin().x(), final_transform.getOrigin().y(), final_transform.getOrigin().z(),
            final_transform.getRotation().x(), final_transform.getRotation().y(), 
            final_transform.getRotation().z(), final_transform.getRotation().w());

        return final_transform;
    }

    // Метод для публикации плоскостей в виде маркеров
    void publish_planes(const std::vector<Cloud::Plane>& planes, const std::string& ns_prefix) {
        if (!planes_pub) return;
        
        visualization_msgs::msg::MarkerArray marker_array;
        
        for (size_t i = 0; i < planes.size(); ++i) {
            const auto& plane = planes[i];
            
            // Создаем маркер для плоскости
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = rclcpp::Clock().now();
            marker.ns = ns_prefix + "_plane";
            marker.id = i;
            marker.type = visualization_msgs::msg::Marker::CUBE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            
            // Устанавливаем позицию и ориентацию
            Eigen::Vector3d centroid = plane.centroid;
            marker.pose.position.x = centroid.x();
            marker.pose.position.y = centroid.y();
            marker.pose.position.z = centroid.z();
            
            // Вычисляем кватернион из нормали плоскости
            Eigen::Vector3d z_axis(0, 0, 1);
            Eigen::Vector3d rotation_axis = z_axis.cross(plane.normal);
            double rotation_angle = std::acos(z_axis.dot(plane.normal));
            
            if (rotation_axis.norm() < 1e-6) {
                // Плоскость уже ориентирована по z
                marker.pose.orientation.x = 0.0;
                marker.pose.orientation.y = 0.0;
                marker.pose.orientation.z = 0.0;
                marker.pose.orientation.w = 1.0;
            } else {
                rotation_axis.normalize();
                Eigen::Quaterniond q(Eigen::AngleAxisd(rotation_angle, rotation_axis));
                marker.pose.orientation.x = q.x();
                marker.pose.orientation.y = q.y();
                marker.pose.orientation.z = q.z();
                marker.pose.orientation.w = q.w();
            }
            
            // Размер плоскости
            const double plane_size = 2.0; // метры
            marker.scale.x = plane_size;
            marker.scale.y = plane_size;
            marker.scale.z = 0.01; // Тонкая плоскость
            
            // Цвет в зависимости от типа плоскости
            if (std::abs(plane.normal.z()) > 0.9) {
                // Горизонтальная плоскость (пол)
                marker.color.r = 0.0;
                marker.color.g = 1.0;
                marker.color.b = 0.0;
                marker.color.a = 0.5;
            } else {
                // Вертикальная плоскость (стена)
                marker.color.r = 1.0;
                marker.color.g = 0.0;
                marker.color.b = 0.0;
                marker.color.a = 0.5;
            }
            
            marker.lifetime = rclcpp::Duration(0, 0); // 0 секунд, 0 наносекунд
            //normal_marker.lifetime = rclcpp::Duration(0, 0); // 0 секунд, 0 наносекунд
            marker_array.markers.push_back(marker);
            
            // Также добавляем маркер для нормали
            visualization_msgs::msg::Marker normal_marker;
            normal_marker.header = marker.header;
            normal_marker.ns = ns_prefix + "_normal";
            normal_marker.id = i;
            normal_marker.type = visualization_msgs::msg::Marker::ARROW;
            normal_marker.action = visualization_msgs::msg::Marker::ADD;
            
            normal_marker.points.resize(2);
            normal_marker.points[0].x = centroid.x();
            normal_marker.points[0].y = centroid.y();
            normal_marker.points[0].z = centroid.z();
            
            // Конец стрелки - нормаль, умноженная на 0.5 метра
            normal_marker.points[1].x = centroid.x() + plane.normal.x() * 0.5;
            normal_marker.points[1].y = centroid.y() + plane.normal.y() * 0.5;
            normal_marker.points[1].z = centroid.z() + plane.normal.z() * 0.5;
            
            normal_marker.scale.x = 0.02; // Диаметр стрелки
            normal_marker.scale.y = 0.04; // Диаметр головки стрелки
            normal_marker.scale.z = 0.0;
            
            normal_marker.color.r = 0.0;
            normal_marker.color.g = 0.0;
            normal_marker.color.b = 1.0;
            normal_marker.color.a = 1.0;
            
            normal_marker.lifetime = rclcpp::Duration::from_seconds(1.0);
            marker_array.markers.push_back(normal_marker);
        }
        
        // Публикуем маркеры
        planes_pub->publish(marker_array);
    }

    void process_clouds() {
        RCLCPP_INFO(rclcpp::get_logger("ICP"), "ICP started");
        
        std::vector<tf2::Transform> transforms;
        transforms.emplace_back(tf2::Transform::getIdentity());
        
        std::vector<sensor_msgs::msg::PointCloud2> processed_wall, processed_marker;
        int cloud_count = 0;

        {
            std::lock_guard<std::mutex> wall_lock(wall_mutex_);
            std::lock_guard<std::mutex> marker_lock(marker_mutex_);
            
            if (wall_pointclouds.empty()) {
                RCLCPP_WARN(rclcpp::get_logger("ICP"), "No point clouds to process");
                return;
            }
            
            processed_wall.push_back(wall_pointclouds[0]);
            processed_marker.push_back(marker_pointclouds[0]);
            wall_result = wall_pointclouds[0];
            marker_result = marker_pointclouds[0];
            cloud_count = wall_pointclouds.size();
        }

        Cloud prev_cloud, current_cloud;
        std::unique_ptr<KDNode> prev_tree;
        std::vector<Eigen::Vector3d> prev_points, current_points;
        
        for(int i = 1; i < cloud_count; i++) {
            if (stop_requested_) {
                RCLCPP_INFO(rclcpp::get_logger("ICP"), "ICP stopped by request");
                return;
            }
            
            RCLCPP_DEBUG(rclcpp::get_logger("ICP"), "Processing ICP %d/%d", i, cloud_count - 1);
            
            // Небольшая задержка для отладки
            if(i == cloud_count - 1) std::this_thread::sleep_for(2000ms);

            sensor_msgs::msg::PointCloud2 past_cloud, current_cloud_msg, marker_cloud;
            {
                std::lock_guard<std::mutex> wall_lock(wall_mutex_);
                std::lock_guard<std::mutex> marker_lock(marker_mutex_);
                
                past_cloud = wall_pointclouds[i - 1];
                current_cloud_msg = wall_pointclouds[i];
                marker_cloud = marker_pointclouds[i];
                cloud_count = wall_pointclouds.size();
            }

            // Преобразуем в точки
            if (i == 1) {
                prev_points = convert_PC_to_vector(past_cloud);
                prev_cloud = Cloud(prev_points);
                // Для point-based ICP используем точки без пола
                std::vector<Eigen::Vector3d> prev_points_no_floor = prev_cloud.get_points_without_floor();
                if (prev_points_no_floor.size() >= 10) {
                    prev_tree = build_kdtree(prev_points_no_floor);
                }
            }
            
            current_points = convert_PC_to_vector(current_cloud_msg);
            current_cloud = Cloud(current_points);

            // Вычисляем трансформацию между текущим и предыдущим кадром
            tf2::Transform incremental_tf;
            
            if (prev_cloud.get_planes().size() >= 1 && current_cloud.get_planes().size() >= 1) {
                // Используем plane-based ICP
                RCLCPP_DEBUG(rclcpp::get_logger("ICP"), "Using plane-based ICP, cloud size: %zu", prev_cloud.get_points().size());
                incremental_tf = ICP_iterations_plane(prev_cloud, current_cloud);
            } else {
                // Fallback на point-based ICP
                RCLCPP_DEBUG(rclcpp::get_logger("ICP"), "Using point-based ICP (fallback)");
                std::vector<Eigen::Vector3d> current_points_no_floor = current_cloud.get_points_without_floor();
                if (current_points_no_floor.size() >= 10000000000 && prev_tree) {
                    run_ICP_Iterations(prev_tree.get(), current_points_no_floor, incremental_tf, 30);
                } else {
                    incremental_tf = tf2::Transform::getIdentity();
                }
            }
            RCLCPP_DEBUG(rclcpp::get_logger("ICP"), 
                "ICP debugging results:"
                "\nResult transform:"
                "\n\t translation: %.3f, %.3f, %.3f"
                "\n\t rotation: %.3f, %.3f, %.3f, %.3f",
                incremental_tf.getOrigin().x(), incremental_tf.getOrigin().y(), incremental_tf.getOrigin().z(),
                incremental_tf.getRotation().x(), incremental_tf.getRotation().y(), 
                incremental_tf.getRotation().z(), incremental_tf.getRotation().w());

            
            // Фильтруем инкрементальную трансформацию
            tf2::Transform filtered_incremental_tf = /*transform_filter_.filter*/(incremental_tf);
            
            filtered_incremental_tf.setIdentity();

            // Обновляем общую трансформацию (относительно первого кадра)
            tf2::Transform global_tf = transforms[i - 1] * filtered_incremental_tf;
            transforms.push_back(global_tf);

            // Преобразуем текущее облако в глобальную систему координат
            sensor_msgs::msg::PointCloud2 transformed_wall = transform_PointCloud(current_cloud_msg, global_tf);
            sensor_msgs::msg::PointCloud2 transformed_marker = transform_PointCloud(marker_cloud, global_tf);
            
            processed_wall.push_back(transformed_wall);
            processed_marker.push_back(transformed_marker);

            // Обновляем результат
            {
                std::lock_guard<std::mutex> wall_lock(wall_mutex_);
                std::lock_guard<std::mutex> marker_lock(marker_mutex_);
                
                concatenate_pointclouds(wall_result, transformed_wall);
                concatenate_pointclouds(marker_result, transformed_marker);
            }
            wall_result.header.frame_id = "map";
            marker_result.header.frame_id = "map";
            // Обновляем для следующей итерации
            prev_cloud = current_cloud;
            prev_points = current_points;
            
            // Обновляем дерево для point-based ICP
            std::vector<Eigen::Vector3d> current_points_no_floor = current_cloud.get_points_without_floor();
            if (current_points_no_floor.size() >= 10) {
                prev_tree = build_kdtree(current_points_no_floor);
            }
            
            // Публикуем промежуточный результат
            publish_result();
            //std::this_thread::sleep_for(1000ms);
            
        }
        
        RCLCPP_INFO(rclcpp::get_logger("ICP"), "ICP ended");
    }

    void publish_result() {
        if (wall_pub) {
            auto wall_msg = get_wall_PC();
            wall_pub->publish(wall_msg);
        }
    }

    void process_loop() {
        while (!stop_requested_) {
            {
                std::lock_guard<std::mutex> lock(wall_mutex_);
                if (wall_pointclouds.size() >= 2) {
                    break;
                }
            }
            std::this_thread::sleep_for(100ms);
        }
        
        if (!stop_requested_) {
            process_clouds();
        }
        
        processing_ = false;
    }
};

class Filter_cloud {
public:
    Filter_cloud() {stop_requested_ = false;}

    void start_processing(ICP* icpd, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr marker_pub) {     
        RCLCPP_INFO(rclcpp::get_logger("FILTER"), "Filtreation initializided");
        this->marker_pub = marker_pub;
        icp = icpd;   
        processing_thread_ = std::thread([this]() {
            process_loop();
        });
    }

    void add_PointCloud(const sensor_msgs::msg::PointCloud2 pc, const geometry_msgs::msg::TransformStamped transform) {
        std::lock_guard<std::mutex> lock(PC_mutex_);
        pointclouds.emplace_back(pc);
        vector_transform.emplace_back(transform);
    }

    void stop_processing() {
        stop_requested_ = true;
        if (processing_thread_.joinable()) {
            processing_thread_.join();
        }
    }

    
private:
    std::thread processing_thread_;
    std::mutex wall_mutex_, marker_mutex_, PC_mutex_;
    std::vector<sensor_msgs::msg::PointCloud2> pointclouds;
    std::vector<geometry_msgs::msg::TransformStamped> vector_transform;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr marker_pub;
    ICP* icp;
    bool stop_requested_;

    void process_loop() {
        int size = 0;
        {
            while(size == 0 && !stop_requested_) {

                {            
                    std::lock_guard<std::mutex> lock(PC_mutex_);
                    size = pointclouds.size();
                }           
                std::this_thread::sleep_for(100ms);
            }
        }
        RCLCPP_INFO(rclcpp::get_logger("ICP"), "Filtreation started");

        for (int i = 0; i < size; i++) {
            if (stop_requested_) break;
            sensor_msgs::msg::PointCloud2 cloud, wall_cloud, marker_cloud;
            while(size - 1 == i && !stop_requested_) {
                {
                    std::lock_guard<std::mutex> lock(PC_mutex_);
                    size = pointclouds.size();
                }
                std::this_thread::sleep_for(100ms);
            }
            {
                std::lock_guard<std::mutex> lock(PC_mutex_);
                cloud = pointclouds[i];
            }
            if(cloud.data.empty()) {
                RCLCPP_ERROR(rclcpp::get_logger("cl"), "Data empty, %d", i);
                continue;
            }
            wall_cloud = cloud;
            marker_cloud = cloud;
            filter_PointCloud(wall_cloud);
            filter_PointCloud(marker_cloud, true);
            transform_PointCloud(wall_cloud, vector_transform[i]);
            transform_PointCloud(marker_cloud, vector_transform[i]);
            icp->add_wall_PC(wall_cloud);
            icp->add_marker_PC(marker_cloud);
            publish_cloud(marker_cloud);
        }
        RCLCPP_INFO(rclcpp::get_logger("ICP"), "Filtreation ended");
    }

    void filter_PointCloud(sensor_msgs::msg::PointCloud2& cloud, bool marker = false) {
        float sortIntensity = 30;
        // 1. Собираем точки с intensity <= 30 и вычисляем centroid
        Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
        std::vector<Eigen::Vector3d> vector_cloud;
        std::vector<float> inten;
        int counter = 0;

        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
        sensor_msgs::PointCloud2Iterator<float> iter_intensity(cloud, "intensity");
        
        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_intensity) {
            if (*iter_intensity <= sortIntensity && !marker) {
                counter++;
                Eigen::Vector3d point(*iter_x, *iter_y, *iter_z);
                centroid += point;
                vector_cloud.emplace_back(point);
                inten.emplace_back(*iter_intensity);
            }
            if (marker && *iter_intensity >= 140 && *iter_intensity <= 160) {
                counter++;
                Eigen::Vector3d point(*iter_x, *iter_y, *iter_z);
                centroid += point;
                vector_cloud.emplace_back(point);
                inten.emplace_back(*iter_intensity);
            }
        }
        
        if (counter == 0) {
            cloud.width = 0;
            cloud.data.clear();
            return;
        }
        
        centroid /= counter;

        // 2. Фильтруем точки по расстоянию от centroid
        std::vector<Eigen::Vector3d> filtered_cloud;
        std::vector<float> filtered_intensity;
        float max_distance = 2;
        if (marker) max_distance = 1;
        for (size_t i = 0; i < vector_cloud.size(); i++) {
            if ((vector_cloud[i]).norm() <= max_distance && vector_cloud[i].norm() >= 0.5) {
                filtered_cloud.emplace_back(vector_cloud[i]);
                filtered_intensity.emplace_back(inten[i]);
            }
        }

        // 3. Обновляем облако точек
        cloud.height = 1;
        cloud.width = filtered_cloud.size();
        
        sensor_msgs::PointCloud2Modifier modifier(cloud);
        modifier.clear();
        modifier.setPointCloud2Fields(4,
            "x", 1, sensor_msgs::msg::PointField::FLOAT32,
            "y", 1, sensor_msgs::msg::PointField::FLOAT32,
            "z", 1, sensor_msgs::msg::PointField::FLOAT32,
            "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
        
        modifier.resize(filtered_cloud.size());
        
        // Используем новые итераторы для записи
        sensor_msgs::PointCloud2Iterator<float> out_x(cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> out_y(cloud, "y");
        sensor_msgs::PointCloud2Iterator<float> out_z(cloud, "z");
        sensor_msgs::PointCloud2Iterator<float> out_intensity(cloud, "intensity");
        
        for (size_t i = 0; i < filtered_cloud.size(); ++i, ++out_x, ++out_y, ++out_z, ++out_intensity) {
            *out_x = static_cast<float>(filtered_cloud[i].x());
            *out_y = static_cast<float>(filtered_cloud[i].y());
            *out_z = static_cast<float>(filtered_cloud[i].z());
            *out_intensity = static_cast<float>(filtered_intensity[i]);
        }
    }

    void transform_PointCloud(sensor_msgs::msg::PointCloud2& cloud, geometry_msgs::msg::TransformStamped& transform) {
        sensor_msgs::msg::PointCloud2 transformed_cloud;
            // Применяем трансформацию
        tf2::doTransform(cloud, transformed_cloud, transform);
        transformed_cloud.header.frame_id = transform.child_frame_id;
        //transformed_cloud.header.stamp = this->now();

        cloud = transformed_cloud;
    }

    void publish_cloud(sensor_msgs::msg::PointCloud2& cloud) {
        if (marker_pub) {
            marker_pub->publish(cloud);
        }
    }

};

class ICP_Node : public rclcpp::Node {
public:
    ICP_Node() : Node("icp_node") {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

        // Инициализация TF buffer и listener (без передачи shared_from_this в конструкторе)
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
            this->get_node_base_interface(),
            this->get_node_timers_interface());
        tf_buffer_->setCreateTimerInterface(timer_interface);
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Параметры для TF
        this->declare_parameter<std::string>("target_frame", "map");
        this->declare_parameter<std::string>("source_frame", "livox");
        target_frame_ = this->get_parameter("target_frame").as_string();
        source_frame_ = this->get_parameter("source_frame").as_string();

        // Подписки и публикации
        wall_sub = create_subscription<sensor_msgs::msg::PointCloud2>(
            "/livox/lidar", qos,
            [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                wall_scan_sub(msg);
            });
                
        wall_pub = create_publisher<sensor_msgs::msg::PointCloud2>("/wall_result", qos);
        marker_pub = create_publisher<sensor_msgs::msg::PointCloud2>("/wall_scans", qos);
        planes_pub = create_publisher<visualization_msgs::msg::MarkerArray>("/detected_planes", qos);

        timer_ = this->create_wall_timer(100ms, [this]() {
            results_pub();
        });

        // Запускаем обработку
        filter.start_processing(&icp, marker_pub);
        icp.start_processing(wall_pub, marker_pub, planes_pub);

        RCLCPP_INFO(this->get_logger(), "ICP Node initialized");
    }

    ~ICP_Node() {
        filter.stop_processing();
        icp.stop_processing();
    }

private:
    ICP icp;
    Filter_cloud filter;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr planes_pub;
    // TF2 компоненты
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::string target_frame_;
    std::string source_frame_;


    void results_pub() {
        if (icp.is_processing()) return;
        
        auto wall_msg = icp.get_wall_PC();
        auto marker_msg = icp.get_marker_PC();
        
        if (wall_msg.data.empty() || marker_msg.data.empty()) return;
        
        wall_msg.header.stamp = this->now();
        marker_msg.header.stamp = this->now();
        
        wall_pub->publish(wall_msg);
        marker_pub->publish(marker_msg);
    }

void wall_scan_sub(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    geometry_msgs::msg::TransformStamped transform;
    
    // Ожидаем трансформацию с таймаутом
    if (wait_for_transform(msg->header.stamp, msg->header.frame_id)) {
        get_transform_to_frame(msg, transform);
        filter.add_PointCloud(*msg, transform);
    } else {
        //RCLCPP_WARN(this->get_logger(), "Transform not available for cloud, skipping");
    }
}

bool wait_for_transform(const rclcpp::Time& stamp, const std::string& frame_id) {
    if (!tf_buffer_) {
        return false;
    }

    if (!tf_buffer_->_frameExists("map")) {
        RCLCPP_DEBUG(this->get_logger(), "Frame 'map' does not exist yet");
        return false;
    }

    try {
        // Ожидаем трансформацию с таймаутом
        return tf_buffer_->canTransform(
            "map", 
            frame_id,
            stamp,
            rclcpp::Duration::from_seconds(0.5)); // 500ms таймаут
    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Transform wait failed: %s", ex.what());
        return false;
    }
}
    void get_transform_to_frame(const sensor_msgs::msg::PointCloud2::SharedPtr msg, geometry_msgs::msg::TransformStamped& transform) {
        // Всегда устанавливаем identity transform по умолчанию
        transform.transform.translation.x = 0;
        transform.transform.translation.y = 0;
        transform.transform.translation.z = 0;
        transform.transform.rotation.x = 0;
        transform.transform.rotation.y = 0;
        transform.transform.rotation.z = 0;
        transform.transform.rotation.w = 1;
        transform.header.frame_id = "map";
        transform.child_frame_id = msg->header.frame_id;
        transform.header.stamp = this->now();

        // Проверяем, доступен ли TF buffer
        if (!tf_buffer_) {
            RCLCPP_ERROR(this->get_logger(), "TF buffer is not initialized");
            return;
        }

        try {
            // Пытаемся получить трансформацию для точного времени сообщения
            transform = tf_buffer_->lookupTransform(
                "map", 
                msg->header.frame_id,
                msg->header.stamp,
                rclcpp::Duration::from_seconds(0.1));
                
            RCLCPP_DEBUG(this->get_logger(), "Successfully got transform for exact time");
            
        } catch (tf2::ExtrapolationException &ex) {
            // Данные LiDAR устарели - используем последнюю доступную трансформацию
            try {
                transform = tf_buffer_->lookupTransform(
                    "map", 
                    msg->header.frame_id,
                    tf2::TimePointZero); // Самый последний доступный
                
                RCLCPP_WARN(this->get_logger(), "Using latest available transform for old LiDAR data");
                
            } catch (tf2::TransformException &ex2) {
                RCLCPP_WARN(this->get_logger(), 
                        "Failed to get latest transform: %s. Using identity.", ex2.what());
            }
            
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "TF transformation failed: %s. Using identity.", ex.what());
        }
    }    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr wall_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr marker_pub;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr wall_sub;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ICP_Node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}