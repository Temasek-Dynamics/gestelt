/**
 * @file polygon.h
 * @brief Polygon class
 */

#ifndef DECOMP_POLYGON_H
#define DECOMP_POLYGON_H

#include <decomp_basis/data_type.h>

namespace Decomp {

  ///Hyperplane class
  template <int Dim>
  struct Hyperplane {
    Hyperplane() {}
    Hyperplane(const Vecf<Dim>& p, const Vecf<Dim>& n) : p_(p), n_(n) {}

    /// Calculate the signed distance from point
    decimal_t signed_dist(const Vecf<Dim>& pt) const {
      return n_.dot(pt - p_);
    }

    /// Calculate the distance from point
    decimal_t dist(const Vecf<Dim>& pt) const {
      return std::abs(signed_dist(pt));
    }

    /// Point on the plane
    Vecf<Dim> p_;
    /// Normal of the plane, directional
    Vecf<Dim> n_;
  };

} // namespace Decomp

///Hyperplane2D: first is the point on the hyperplane, second is the normal
typedef Decomp::Hyperplane<2> Hyperplane2D;
///Hyperplane3D: first is the point on the hyperplane, second is the normal
typedef Decomp::Hyperplane<3> Hyperplane3D;

///Polyhedron class
template <int Dim>
struct Polyhedron {

  ///Null constructor
  Polyhedron() {}
  ///Construct from Hyperplane array
  Polyhedron(const vec_E<Decomp::Hyperplane<Dim>>& vs) : vs_(vs) {}


  ///Append Hyperplane
  void add(const Decomp::Hyperplane<Dim>& v) {
    vs_.push_back(v);
  }

  /// Check if the point is inside polyhedron, non-exclusive
  bool inside(const Vecf<Dim>& pt) const {
    for (const auto& v : vs_) {
      if (v.signed_dist(pt) > epsilon_) {
        //printf("rejected pt: (%f, %f), d: %f\n",pt(0), pt(1), v.signed_dist(pt));
        return false;
      }
    }
    return true;
  }

  /// Calculate points inside polyhedron, non-exclusive
  vec_Vecf<Dim> points_inside(const vec_Vecf<Dim> &O) const {
    vec_Vecf<Dim> new_O;
    for (const auto &it : O) {
      if (inside(it))
        new_O.push_back(it);
    }
    return new_O;
  }

  /// Calculate normals, used for visualization
  vec_E<std::pair<Vecf<Dim>, Vecf<Dim>>> cal_normals() const {
    vec_E<std::pair<Vecf<Dim>, Vecf<Dim>>> ns(vs_.size());
    for (size_t i = 0; i < vs_.size(); i++)
      ns[i] = std::make_pair(vs_[i].p_, vs_[i].n_); // fist is point, second is normal
    return ns;
  }

  /// Get the hyperplane array
  vec_E<Decomp::Hyperplane<Dim>> hyperplanes() const {
    return vs_;
  }

  /// Hyperplane array
  vec_E<Decomp::Hyperplane<Dim>> vs_; // normal must go outside

};

///Polyhedron2D, consists of 2D hyperplane
typedef Polyhedron<2> Polyhedron2D;
///Polyhedron3D, consists of 3D hyperplane
typedef Polyhedron<3> Polyhedron3D;

///[A, b] for \f$Ax < b\f$
template <int Dim>
struct LinearConstraint {
  
  ///Null constructor
  LinearConstraint() {}
  /// Construct from \f$A, b\f$ directly, s.t \f$Ax < b\f$
  LinearConstraint(const MatDNf<Dim>& A, const VecDf& b) : A_(A), b_(b) {
    Eigen::MatrixX4d hyp_mat(A.rows(), Dim+1); // Size (num_vertices, 4)

		for (unsigned int i = 0; i < A.rows(); i++) {
      hyp_mat.block<1, Dim+1>(i, 0) << A.row(i) , -b(i); // Size (num_vertices, 4)
		}

		hyp_mat_ = hyp_mat;
  }
  
  /**
   * @brief Construct from a inside point and hyperplane array
   * @param p0 point that is inside
   * @param vs hyperplane array, normal should go outside
   */
	LinearConstraint(const Vecf<Dim> p0, const vec_E<Decomp::Hyperplane<Dim>>& vs) {
		const unsigned int size = vs.size();
		MatDNf<Dim> A(size, Dim);
		VecDf b(size);

    Eigen::MatrixX4d hyp_mat(size, Dim+1); // Size (num_vertices, 4)

		for (unsigned int i = 0; i < size; i++) {
			auto n = vs[i].n_;
			decimal_t c = vs[i].p_.dot(n);
			if (n.dot(p0) - c < 0) { // Point p0 is inside but n is pointing the other way 
				n = -n;
				c = -c;
			}

      // We want all the halfspaces to be in the form of:
      //    n_x * x + n_y * y + n_z * z - c <= 0 
      // OR A(0)*x + A(1)*y + A(2)*z - b <= 0
			// if (n.dot(p0) - c > 0) { 
			// 	n = -n;
			// 	c = -c;
			// }

			A.row(i) = n;
			b(i) = c;

      hyp_mat.block<1, Dim+1>(i, 0) << n, -c; // Size (num_vertices, 4)
		}

		A_ = A;
		b_ = b;
		hyp_mat_ = hyp_mat;
	}

  /// Check if the point is inside polyhedron using linear constraint
  bool inside(const Vecf<Dim> &pt) {
    VecDf d = A_ * pt - b_;
    for (unsigned int i = 0; i < d.rows(); i++) {
      if (d(i) > 0)
        return false;
    }
    return true;
  }

  // returns matrix of size (N, 4), representing polyhedrons with half-planes
  Eigen::MatrixX4d getHypMatrix() const {
    return hyp_mat_;
  }

  /// Get \f$A\f$ matrix
  MatDNf<Dim> A() const { return A_; }

  /// Get \f$b\f$ matrix
  VecDf b() const { return b_; }

  MatDNf<Dim> A_;
  VecDf b_;

  Eigen::MatrixX4d hyp_mat_; // matrix of size (N, 4), representing polyhedrons with half-planes
};

///LinearConstraint 2D
typedef LinearConstraint<2> LinearConstraint2D;
///LinearConstraint 3D
typedef LinearConstraint<3> LinearConstraint3D;

#endif
