#include "main.h"
#include "monte.hpp"

const int PARTICLE_QUANTITY = 7500;

std::vector<particle> particles;

std::random_device rd;

std::mt19937 gen(rd());


std::normal_distribution<float> noise_x(0.0f,
                                        0.1f);
std::normal_distribution<float> noise_y(0.0f,
                                        0.1f);
std::normal_distribution<float> noise_theta(0.0f, 5.0f);

Point lastPoint(0,0,0);

const int MCL_delay = 20;
const float feild_dim = 144.0f;

const float NORTH_SENSOR_X_OFFSET = 0.0f;
const float NORTH_SENSOR_Y_OFFSET = 0.0f;

const float SOUTH_SENSOR_X_OFFSET = 0.0f;
const float SOUTH_SENSOR_Y_OFFSET = 0.0f;

const float EAST_SENSOR_X_OFFSET = 0.0f;
const float EAST_SENSOR_Y_OFFSET = 0.0f;

const float WEST_SENSOR_X_OFFSET = 0.0f; 
const float WEST_SENSOR_Y_OFFSET = 0.0f;

bool usingNorth = false;
bool usingSouth = false;
bool usingWest = false;
bool usingEast = true;


const float sigma_close_range =
    0.3f; // Sigma for distances below 200mm (approx 7.87 inches)
const float sigma_far_range = 1.0f; // Sigma for distances above 200mm
const float DISTANCE_THRESHOLD_INCHES = 7.87f; // 200mm in inches

// Add variables for update frequency control
int lastUpdateTime = 0;           // Timestamp of the last MCL update
const int UPDATE_INTERVAL = 1000; // Update interval in milliseconds (1 second)
Point lastUpdatedPose(0, 0, 0); // Last pose at which MCL was updated
const float MIN_MOTION_THRESHOLD =
    0.25f; // Minimum motion in inches to trigger update

// Add constant for motion noise threshold
const float MOTION_NOISE_THRESHOLD =
    0.25f; // Threshold to consider motion as static

const float UNIFORM_WEIGHT_FACTOR =
    0.0001f;                   // Small uniform weight factor, tune as needed
const float MIN_WEIGHT = 0.1f; // Minimum weight floor for particles
const int RESAMPLING_INTERVAL = 3; // Only resample every 3rd update

// Add variables to store previous sensor readings
float prev_dist = -1.0f;
const float DISTANCE_CHANGE_THRESHOLD = 0.25f; // Adjust based on sensor noise

// Add this at the global scope
Point filteredPose(0, 0, 0); // {{ Added filteredPose }}
const float FILTER_ALPHA =
    0.3f; // Smoothing factor (0.3 = 30% new, 70% old) {{ Added FILTER_ALPHA }}
const float ODOMETRY_TRUST_FACTOR =
    0.5f; // 0.5 = equal trust in odometry and sensors

void initalizeParticles(Point inital){
    particles.resize(PARTICLE_QUANTITY);
    lastPoint = inital;

    std::normal_distribution<float> x_dist(initialPose.x, 1.0);
    std::normal_distribution<float> y_dist(initialPose.y, 1.0);
    std::normal_distribution<float> theta_dist(initialPose.theta, 0.0);

    for (auto &particle : particles) {
        particle = Particle(point(x_dist(gen), y_dist(gen), theta_dist(gen)),
                        1.0f / PARTICLE_QUANTITY);
  }
}

void motionUpdate(Point poseDelta){
    float Magnitude = std::sqrt(poseDelta.x*poseDelta.x + poseDelta.y * poseDelta.y);

    bool add_noise = Magnitude > 0.5f; // how much the robot has to move to initate the noise

    std::normal_distribution<float> motion_noise(0.0, 0.03);
    std::normal_distribution<float> rotation_noise(0.0, 0.05);


    for (auto &particle : particles) {
        float theta_rad = particle.theta * M_PI / 180.0f;
        float dx_global =
            localOdomDelta.x * cos(theta_rad) - localOdomDelta.y * sin(theta_rad);
        float dy_global =
            localOdomDelta.x * sin(theta_rad) + localOdomDelta.y * cos(theta_rad);

        float noise_scale = add_noise ? std::min(1.0f, motion_magnitude / 2.0f): 0.0f;
        
        particle.x +=
            dx_global +
            noise_scale * motion_noise(gen); // Apply scaled motion noise
        particle.y +=
            dy_global +
            noise_scale * motion_noise(gen); // Apply scaled motion noise
        particle.theta +=
            localOdomDelta.theta +
            noise_scale * rotation_noise(gen); // Apply scaled rotation noise

    // Normalize theta to be within 0-360 degrees
    particle.pose.theta = fmod(
        particle.pose.theta, 360.0f);
    if (particle.pose.theta < 0) {
      particle.pose.theta += 360.0f;
    }
    }
}

float predictParticle(Point par, char dir){
  float half_dimension = FIELD_DIMENSIONS / 2.0f;
  float sensor_x = par.x;
  float sensor_y = par.y;
  float theta = par.theta * M_PI / 180.0f; // Robot orientation in radians

  // Define the sensor's FOV (±12° from center)
  //We do this becuase the vex Distance sensor reading is the nearest object in a cone
  const float FOV_HALF_ANGLE = 12.0f * M_PI / 180.0f;

  // Direction-specific center angle (assuming 0° is north)
  float sensor_angle = 0.0f;
  switch (dir) {
  case 'N':
    sensor_angle = 0.0f;
    break; // Up (y = 72)
  case 'S':
    sensor_angle = M_PI;
    break; // Down (y = -72)
  case 'E':
    sensor_angle = M_PI / 2.0f;
    break; // Right (x = 72)
  case 'W':
    sensor_angle = 3.0f * M_PI / 2.0f;
    break; // Left (x = -72)
  default:
    return -1.0f;
  }

  float center_angle = sensor_angle + theta;


  float left_angle = center_angle - FOV_HALF_ANGLE;
  float right_angle = center_angle + FOV_HALF_ANGLE;

  float dist[4];

  float north_left = (half_dimension-sensor_y) / sin(left_angle);
  float north_right = (half_dimension-sensor_y) / sin(right_angle);
  
  dist[0] = (north_left > 0 && std::isfinite(north_left))? north_left : std::numeric_limits<float>::max();
  dist[0] = std::min(dist[0], (north_right > 0 && std::isfinite(north_right))? north_right : std::numeric_limits<float>::max();

  float south_left = (-half_dimension-sensor_y) / sin(left_angle);
  float south_right = (-half_dimension-sensor_y) / sin(right_angle);
  
  dist[1] = (south_left > 0 && std::isfinite(south_left))? south_left : std::numeric_limits<float>::max();
  dist[1] = std::min(dist[1], (south_right > 0 && std::isfinite(south_right))? south_right : std::numeric_limits<float>::max();

  float east_left = (half_dimension-sensor_x) / sin(left_angle);
  float east_right = (half_dimension-sensor_x) / sin(right_angle);
  
  dist[2] = (east_left > 0 && std::isfinite(east_left))? east_left : std::numeric_limits<float>::max();
  dist[2] = std::min(dist[1], (east_right > 0 && std::isfinite(east_right))? east_right : std::numeric_limits<float>::max();

  float west_left = (-half_dimension-sensor_x) / sin(left_angle);
  float west_right = (-half_dimension-sensor_x) / sin(right_angle);
  
  dist[3] = (west_left > 0 && std::isfinite(west_left))? west_left : std::numeric_limits<float>::max();
  dist[3] = std::min(dist[1], (west_right > 0 && std::isfinite(west_right))? west_right : std::numeric_limits<float>::max();

  float minDist = std::numeric_limits<float>::max();
  for (int i = 0; i < 4;i++){
    if (dist[i] > 0 && dist[i] < minDist){
        minDist = dist[i];
    }
  }

  if(minDist == std::numeric_limits<float>::max()){
    minDist = 72.0f;
  }

  return minDist;


}

Point PtoP(p){
    retun Point(p.x,p.y,p.theta);
}

void measurementUpdate(float dist){
    bool sigChange = false;
    if (dist >= 0&& prev_dist >= 0&& std::abs(dist-prev_dist) > DISTANCE_CHANGE_THRESHOLD){
        sigChange = true;
    }
    if(sigChange == false){
        prev_dist = dist;
        return;
    }

    float totalWeight = 0.0f;
    

    for (auto &particle : particles){
        float particle_weight = 1.0f;
        int valid_readings = 0;
        float sigma = 10.0f;

        if (dist >= 0){
            float pred_dist = predictParticle(PtoP(particle));
            float diff = std::abs(pred_dist-dist);
            float likelihood = std::exp(-(diff*diff)/(2.0f*sigma*sigma));
            particle_weight *= likelihood;
            valid_readings ++;
        }
        if (valid_readings > 0){
            particle.weight = std::max(particle_weight,0.1f);
        }else{
            particle.weight = 0.1f;
        }
        totalWeight += particle.weight;
    }

    if(totalWeight > 0){
        for (auto &particle : particles){
            particle.weight /= totalWeight;
        }
    }
    prev_dist = dist;
}


std::vector<float> weightedResample(std::vector<float> &particles){

}