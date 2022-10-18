#ifndef QUATERNION_HPP
#define QUATERNION_HPP

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <cmath>

namespace digital_twin {
  
  template<typename T> class quaternion;

  /*
   * Apply the rotation represented by quaternion q to the vector v. 
   */
  template<typename T>
  glm::vec3 operator*(const quaternion<T>& q, const glm::vec3& v);
  
  template<typename T>
  quaternion<T>& operator*(const quaternion<T>& q1, const quaternion<T>& q2);
  
  /*
   * Quaternion class. The real part is w. 
   */
  template<typename T> class quaternion {
  public:

    // default construct a real unit quaternion
    quaternion()
      : x_{0.0}, y_{0.0}, z_{0.0}, w_{1.0} { }

    // constructor for given values
    quaternion(T x, T y, T z, T w)
      : x_{x}, y_{y}, z_{z}, w_{w} { }

    /*
     * Construct a pure-imaginary quaternion from the the given
     * vector.
     */
    quaternion(const glm::vec3& vec)
      : x_{vec.x}, y_{vec.y}, z_{vec.z}, w_{0.0} { }

    quaternion(const quaternion<T>& other) = default;
    
    quaternion(const glm::vec3 axis, T angle) {
      T sin_half_angle = std::sin(angle / 2.0);
      T cos_half_angle = std::cos(angle / 2.0);

      x_ = sin_half_angle * axis.x;
      y_ = sin_half_angle * axis.y;
      z_ = sin_half_angle * axis.z;
      w_ = cos_half_angle;      
    }
    
    // accessors
    inline T x() const { return x_; }
    inline T y() const { return y_; }
    inline T z() const { return z_; }
    inline T w() const { return w_; }

    inline T norm_sq() const {
      return x_ * x_ + y_ * y_ + z_ * z_ + w_ * w_;
    }

    inline T norm() const {
      return std::sqrt(this->norm_sq());
    }

    void normalize();      

    // get the real part of this quaternion
    inline T real() const {
      return w_;
    }

    // get the pure imaginary part of this quaternion
    inline glm::vec3 imag() const {
      return glm::vec3(x_, y_, z_);
    }
    
    // get the angle represented by this quaternion
    // precondition - *this should be a unit quaternion
    inline T get_angle() const {
      return 2.0 * std::acos(w_);
    }

    inline glm::vec3 get_axis() const {
      glm::vec3 a(x_, y_, z_);
      T theta = this->get_angle();
      return (1.0 / sin(theta/2.0)) * a;
    }

    // precondition - *this should be a unit quaternion.
    quaternion<T> conjugate() const {
      return quaternion<T>{-x_, -y_, -z_, w_};
    }

    // precondition - *this should be a unit quaternion.
    inline quaternion<T> inv() const {
      return this->conjugate();
    }

    inline quaternion<T> power(T alpha) const {
      T angle = this->get_angle();
      glm::vec3 axis = this->get_axis();

      return quaternion<T>{axis, (alpha * angle)/2.0};
    }

    // get the rotation matrix for this quaternion.
    // precondition - *this should be a unit quaternion.
    glm::mat4 rotation_matrix() const;
    
    quaternion<T>& operator*=(const quaternion<T>& other);
    quaternion<T>& operator/=(T c);

    friend glm::vec3 operator*<T>(const quaternion<T>& q, const glm::vec3& v);
    friend quaternion<T>& operator*<T>(const quaternion<T>& q1, const quaternion<T>& q2);
    
  private:
    T x_;
    T y_;
    T z_;
    T w_;
    
  };

#include "./quaternion_impl.hpp"
  
} // namespace digital_twin


#endif // QUATERNION_HPP
