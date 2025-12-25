#ifndef KALMAN_POSITIONING_LANDMARK_MANAGER_HPP
#define KALMAN_POSITIONING_LANDMARK_MANAGER_HPP

#include <map>
#include <string>
#include <vector>
#include <utility>

class LandmarkManager {
public:
    /**
     * @brief Load landmarks from CSV file
     * Expected format: id,x,y
     * @param csv_path Path to the CSV file
     * @return true if successfully loaded, false otherwise
     */
    bool loadFromCSV(const std::string& csv_path);
    
    /**
     * @brief Get all landmarks
     * @return Map of landmark_id -> {x, y}
     */
    const std::map<int, std::pair<double, double>>& getLandmarks() const;
    
    /**
     * @brief Get landmark position by ID
     * @param id Landmark ID
     * @return {x, y} or empty pair if not found
     */
    std::pair<double, double> getLandmark(int id) const;
    
    /**
     * @brief Check if landmark exists
     */
    bool hasLandmark(int id) const;
    
    /**
     * @brief Get number of landmarks
     */
    size_t getNumLandmarks() const { return landmarks_.size(); }
    
    /**
     * @brief Get landmarks within a certain radius of a point
     * @param x, y Center point
     * @param radius Search radius
     * @return Vector of landmark IDs within radius
     */
    std::vector<int> getLandmarksInRadius(double x, double y, double radius) const;
    
    /**
     * @brief Calculate distance between two points
     */
    static double distance(double x1, double y1, double x2, double y2);

private:
    std::map<int, std::pair<double, double>> landmarks_;  // id -> {x, y}
};

#endif  // KALMAN_POSITIONING_LANDMARK_MANAGER_HPP
