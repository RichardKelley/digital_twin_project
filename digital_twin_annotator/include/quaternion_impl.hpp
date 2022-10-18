#include <cmath>


  template<typename T>
  quaternion<T>& quaternion<T>::operator*=(const quaternion<T>& other) {
    // TODO
    return *this;
  }

  template <typename T>
  quaternion<T>& quaternion<T>::operator/=(T c) {
    this->x_ /= c;
    this->y_ /= c;
    this->z_ /= c;
    this->w_ /= c;
    
    return *this;
  }
  
  template<typename T>
  void quaternion<T>::normalize() {
    T n = this->norm();
    if (std::abs(n) > 0.0) {
      x_ /= n;
      y_ /= n;
      z_ /= n;
      w_ /= n;
    }
  }

  template<typename T>
  glm::mat4 quaternion<T>::rotation_matrix() const {
    T m11 = 1.0 - 2.0 * y_ * y_ - 2.0 * z_ * z_;
    T m12 = 2.0 * x_ * y_ - 2.0 * w_ * z_;
    T m13 = 2.0 * x_ * z_ + 2.0 * w_ * y_;

    T m21 = 2.0 * x_ * y_ + 2.0 * w_ * z_;
    T m22 = 1.0 - 2.0 * x_ * x_ - 2.0 * z_ * z_;
    T m23 = 2.0 * y_ * z_ - 2.0 * w_ * x_;

    T m31 = 2.0 * x_ * z_ - 2.0 * w_ * y_;
    T m32 = 2.0 * y_ * z_ + 2.0 * w_ * x_;
    T m33 = 1.0 - 2.0 * x_ * x_ - 2.0 * y_ * y_;

    return glm::mat4(m11, m21, m31, 0.0,
		     m12, m22, m32, 0.0,
		     m13, m23, m33, 0.0,
		     0.0, 0.0, 0.0, 1.0);
  }
  
  template<typename T>
  glm::vec3 operator*(const quaternion<T>& q, const glm::vec3& v) {
    // convert v to a pure imaginary quaternion
    quaternion<T> v_quat{v};
    
    // perform conjugation
    quaternion<T> out = q * v_quat * q.inv();

    // return vec3
    return out.imag();
  }

  template<typename T>
  quaternion<T> operator*(const quaternion<T>& q1, const quaternion<T>& q2) {
    T w = q1.w_ * q2.w_ - q1.x_ * q2.x_ - q1.y_ * q2.y_ - q1.z_ * q2.z_;
    T x = q1.w_ * q2.x_ + q1.x_ * q2.w_ + q1.y_ * q2.z_ - q1.z_ * q2.y_;    
    T y = q1.w_ * q2.y_ - q1.x_ * q2.z_ + q1.y_ * q2.w_ + q1.z_ * q2.x_;
    T z = q1.w_ * q2.z_ + q1.x_ * q2.y_ - q1.y_ * q2.x_ + q1.z_ * q2.w_;
    
    return quaternion<T>{x,y,z,w};
  }



