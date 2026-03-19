#include <Adafruit_FXAS21002C.h>
#include <Adafruit_FXOS8700.h>
#include <Adafruit_Sensor.h>
#include <math.h>

Adafruit_FXOS8700 accelmag = Adafruit_FXOS8700(0x8700A, 0x8700B);
Adafruit_FXAS21002C gyro = Adafruit_FXAS21002C(0x0021002C);

// time increment
int pev_ms; double dt;
// default vals
double gyro_off_x = 0.0056435823, gyro_off_y = 0.0056435823, gyro_off_z = -0.00029412741;
double gx_noise = 0.0035897517, gy_noise = 0.0039893616, gz_noise = 0.0027715511;
double ax_noise = 0.027461054, ay_noise = 0.022408009, az_noise = 0.039931731;
double gravity_magnitude = 9.82588196; // Thailand Default
double roll, pitch, yaw;

// gaussian filter
#define WINDOW_SIZE 9
class AxisFilter {
  private:
    double buffer[WINDOW_SIZE];
    const double kernel[WINDOW_SIZE] = {
      0.0162, 0.0540, 0.1210, 0.1942, 0.2292, 0.1942, 0.1210, 0.0540, 0.0162
    };   
    double noise_threshold;

  public:
    AxisFilter() {
      for(int i=0; i<WINDOW_SIZE; i++) buffer[i] = 0.0;
      noise_threshold = 0.02; 
    }

    void setNoiseThreshold(double measured_noise_std_dev) {
      noise_threshold = measured_noise_std_dev * 4.0; 
    }

    double update(double raw_input) {
      for (int i = 0; i < WINDOW_SIZE - 1; i++) {
        buffer[i] = buffer[i+1];
      }
      buffer[WINDOW_SIZE - 1] = raw_input;

      double smooth_val = 0.0;
      for (int i = 0; i < WINDOW_SIZE; i++) {
        smooth_val += buffer[i] * kernel[i];
      }

      double mean = 0.0;
      for (int i = 0; i < WINDOW_SIZE; i++) mean += buffer[i];
      mean /= WINDOW_SIZE;

      double variance = 0.0;
      for (int i = 0; i < WINDOW_SIZE; i++) variance += pow(buffer[i] - mean, 2);
      double current_std_dev = sqrt(variance / WINDOW_SIZE);

      double mix = (current_std_dev - noise_threshold) / noise_threshold; 
      
      if (mix < 0.0) mix = 0.0;
      if (mix > 1.0) mix = 1.0;

      return (raw_input * mix) + (smooth_val * (1.0 - mix));
    }
    
    // Function to check stillness
    // bool isStable() {
    //   double mean = 0.0;
    //   for (int i = 0; i < WINDOW_SIZE; i++) mean += buffer[i];
    //   mean /= WINDOW_SIZE;
    //   double variance = 0.0;
    //   for (int i = 0; i < WINDOW_SIZE; i++) variance += pow(buffer[i] - mean, 2);
    //   return sqrt(variance / WINDOW_SIZE) < noise_threshold;
    // }
};
AxisFilter filterAx, filterAy, filterAz;
AxisFilter filterGx, filterGy, filterGz;

void calibrateSensors() {
  Serial.println("Calibrating, DO NOT MOVE ~ 15 seconds");
  
  int samples = 1080*3;
  double ax_sum=0, ay_sum=0, az_sum=0, gx_sum=0, gy_sum=0, gz_sum=0;
  double ax_sq_sum=0, ay_sq_sum=0, az_sq_sum=0, gx_sq_sum=0, gy_sq_sum=0, gz_sq_sum=0;

  for(int i=0; i<samples; i++) {
    sensors_event_t gevent, aevent, mevent;
    gyro.getEvent(&gevent);
    accelmag.getEvent(&aevent, &mevent);
    
    double gx = gevent.gyro.x; double gy = gevent.gyro.y; double gz = gevent.gyro.z;
    double ax = aevent.acceleration.x; double ay = aevent.acceleration.y; double az = aevent.acceleration.z;
    
    gx_sum += gx; gy_sum += gy; gz_sum += gz;
    ax_sum += ax; ay_sum += ay; az_sum += az;

    gx_sq_sum += gx*gx; gy_sq_sum += gy*gy; gz_sq_sum += gz*gz;
    ax_sq_sum += ax*ax; ay_sq_sum += ay*ay; az_sq_sum += az*az;

    delay(2);
  }
  
  gyro_off_x = gx_sum / samples;
  gyro_off_y = gy_sum / samples;
  gyro_off_z = gz_sum / samples;
  
  double ax_mean = ax_sum / samples;
  double ay_mean = ay_sum / samples;
  double az_mean = az_sum / samples; 

  gravity_magnitude = sqrt(ax_mean * ax_mean + ay_mean * ay_mean + az_mean * az_mean);

  for(int i=0; i<samples; i++) {
    sensors_event_t gevent, aevent, mevent;
    gyro.getEvent(&gevent);
    accelmag.getEvent(&aevent, &mevent);

    delay(2);
  }

  gx_noise = sqrt(abs((gx_sq_sum / samples) - (gyro_off_x * gyro_off_x)));
  gy_noise = sqrt(abs((gy_sq_sum / samples) - (gyro_off_y * gyro_off_y)));
  gz_noise = sqrt(abs((gz_sq_sum / samples) - (gyro_off_z * gyro_off_z)));
  
  ax_noise = sqrt(abs((ax_sq_sum / samples) - (ax_mean * ax_mean)));
  ay_noise = sqrt(abs((ay_sq_sum / samples) - (ay_mean * ay_mean)));
  az_noise = sqrt(abs((az_sq_sum / samples) - (az_mean * az_mean)));


  // Printing results for default values
  Serial.print("Gyro Offsets: ");
  Serial.print(gyro_off_x, 16); Serial.print(", ");
  Serial.print(gyro_off_y, 16); Serial.print(", ");
  Serial.print(gyro_off_z, 16); Serial.print(", ");
  Serial.print(gx_noise, 16); Serial.print(", ");
  Serial.print(gy_noise, 16); Serial.print(", ");
  Serial.println(gz_noise, 16);
  Serial.print(ax_noise, 16); Serial.print(", ");
  Serial.print(ay_noise, 16); Serial.print(", ");
  Serial.println(az_noise, 16);
  Serial.println(gravity_magnitude, 16);

  Serial.println("Calibration Done.");
}

class Madgwick {
  public:
    double beta; 
    double q0, q1, q2, q3; 
    double radToDeg;

    Madgwick() {
      beta = 0.05; 
      q0 = 1.0; q1 = 0.0; q2 = 0.0; q3 = 0.0;
      radToDeg = 57.29577951308232;
    }

    void update(double gx, double gy, double gz, double ax, double ay, double az, double dt) {
      double recipNorm;
      double s0, s1, s2, s3;
      double qDot1, qDot2, qDot3, qDot4;
      double _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

      double accel_mag = sqrt(ax*ax + ay*ay + az*az);
      
      double effective_beta = beta;
      double motion_offset = abs(accel_mag - gravity_magnitude);
      if (motion_offset > 2.0) { 
          effective_beta = 0.0; 
      } else {
          effective_beta = beta * (1.0 - (motion_offset / 2.0)); 
      }

      qDot1 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz);
      qDot2 = 0.5 * (q0 * gx + q2 * gz - q3 * gy);
      qDot3 = 0.5 * (q0 * gy - q1 * gz + q3 * gx);
      qDot4 = 0.5 * (q0 * gz + q1 * gy - q2 * gx);

      if(!((ax == 0.0) && (ay == 0.0) && (az == 0.0))) {

        recipNorm = 1.0 / accel_mag; 
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        _2q0 = 2.0 * q0; _2q1 = 2.0 * q1; _2q2 = 2.0 * q2; _2q3 = 2.0 * q3;
        _4q0 = 4.0 * q0; _4q1 = 4.0 * q1; _4q2 = 4.0 * q2;
        _8q1 = 8.0 * q1; _8q2 = 8.0 * q2;
        q0q0 = q0 * q0; q1q1 = q1 * q1; q2q2 = q2 * q2; q3q3 = q3 * q3;

        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0 * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0 * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0 * q1q1 * q3 - _2q1 * ax + 4.0 * q2q2 * q3 - _2q2 * ay;
        
        recipNorm = 1.0 / sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); 
        s0 *= recipNorm; s1 *= recipNorm; s2 *= recipNorm; s3 *= recipNorm;

        qDot1 -= effective_beta * s0;
        qDot2 -= effective_beta * s1;
        qDot3 -= effective_beta * s2;
        qDot4 -= effective_beta * s3;
      }

      q0 += qDot1 * dt;
      q1 += qDot2 * dt;
      q2 += qDot3 * dt;
      q3 += qDot4 * dt;

      recipNorm = 1.0 / sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
      q0 *= recipNorm;
      q1 *= recipNorm;
      q2 *= recipNorm;
      q3 *= recipNorm;
    }

    void computeAngles(double &roll, double &pitch, double &yaw) {
      roll = atan2(q0*q1 + q2*q3, 0.5 - q1*q1 - q2*q2) * radToDeg;
      pitch = asin(-2.0 * (q1*q3 - q0*q2)) * radToDeg;
      yaw = atan2(q1*q2 + q0*q3, 0.5 - q2*q2 - q3*q3) * radToDeg;
    }
};

// Madgwick
Madgwick madgwick;

void setup(void) {
  Serial.begin(115200);
  while (!Serial) delay(1);
  if (!gyro.begin()) while (1);
  if (!accelmag.begin()) while (1);
  accelmag.setOutputDataRate(ODR_200HZ);
  // calibrateSensors();  // Optional
  filterGx.setNoiseThreshold(gx_noise);
  filterGy.setNoiseThreshold(gy_noise);
  filterGz.setNoiseThreshold(gz_noise);
  filterAx.setNoiseThreshold(ax_noise);
  filterAy.setNoiseThreshold(ay_noise);
  filterAz.setNoiseThreshold(az_noise);
  pev_ms = millis();
}

void loop(void) {
  int current_ms = millis();
  dt = (current_ms - pev_ms) / 1000.0;
  pev_ms = current_ms;

  sensors_event_t gevent, aevent, mevent;
  gyro.getEvent(&gevent);
  accelmag.getEvent(&aevent, &mevent);

  double gx = filterGx.update(gevent.gyro.x - gyro_off_x);
  double gy = filterGy.update(gevent.gyro.y - gyro_off_y);
  double gz = filterGz.update(gevent.gyro.z - gyro_off_z);
  double ax = filterAx.update(aevent.acceleration.x);
  double ay = filterAy.update(aevent.acceleration.y);
  double az = filterAz.update(aevent.acceleration.z);

  madgwick.update(gx, gy, gz, ax, ay, az, dt);
  madgwick.computeAngles(roll, pitch, yaw);

  Serial.print(roll, 2);
  Serial.print(",");
  Serial.print(pitch, 2);
  Serial.print(",");
  Serial.println(yaw, 2);
  
  delay(5);
}