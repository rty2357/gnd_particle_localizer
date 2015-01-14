/**
 * @file /include/gnd_particle_localizer/gnd_particle_localizer.hpp
 *
 * @brief Template class for gnd_particle_localizer.
 **/
#ifndef gnd_particle_localizer_HPP
#define gnd_particle_localizer_HPP

#ifdef WIN32
#ifdef gnd_particle_localizer_EXPORTS
#define gnd_particle_localizer_API __declspec(dllexport)
#else
#define gnd_particle_localizer_API __declspec(dllimport)
#endif
#else
#define gnd_particle_localizer_API
#endif

#include "gnd_msgs/msg_pose2d_stamped.h"
#include "gnd_msgs/msg_velocity2d_with_covariance_stamped.h"
#include "gnd_particle_localizer/msg_particles_pose2d_stamped.h"
#include "gnd_particle_localizer/msg_particle_weights_stamped.h"
#include "gnd_particle_localizer/srv_reset_particles_normal_distribution.h"

#include "gnd/gnd_rosmsg_reader.hpp"
#include "gnd/gnd-vector-base.hpp"

#include <float.h>

// ---> type declaration ( callback function object )
namespace gnd {
	namespace particle_localizer {
		class srv_funcobj_reset_particles_normal_distribution;

	}
} // <--- type declaration ( callback function object )


// ---> message type definition
namespace gnd {
	namespace particle_localizer {
		typedef gnd_msgs::msg_pose2d_stamped											msg_pose_t;
		typedef gnd_msgs::msg_velocity2d_with_covariance_stamped						msg_motion_t;
		typedef gnd::rosutil::rosmsgs_reader_stamped<msg_motion_t>						msgreader_motion_t;
		typedef gnd_particle_localizer::msg_particles_pose2d_stamped					msg_particles_t;
		typedef msg_particles_t::_poses_type											particles_t;
		typedef particles_t::value_type													particle_t;
		typedef gnd_particle_localizer::msg_particle_weights_stamped					msg_particle_weights_t;
		typedef gnd::rosutil::rosmsgs_reader_stamped<msg_particle_weights_t>			msgreader_particle_weights_t;
		typedef msg_particle_weights_t::_weights_type									particle_weights_t;
		typedef particle_weights_t::value_type											particle_weight_t;
		typedef gnd::vector::fixed_column<3> 											systematic_error_ratio_t;

		typedef gnd_particle_localizer::srv_reset_particles_normal_distributionRequest	srv_reset_particles_nd_request_t;
		typedef gnd_particle_localizer::srv_reset_particles_normal_distributionResponse srv_reset_particles_nd_response_t;

		// ---> callback function object
		class srv_funcobj_reset_particles_normal_distribution {
		public:
			typedef srv_reset_particles_nd_request_t request_t;
			typedef srv_reset_particles_nd_response_t response_t;

		private:
			struct {
				double x;
				double y;
				double theta;
			} average_;
			double covariance_[9];
			int size_;
			bool is_called_;
		public:
			srv_funcobj_reset_particles_normal_distribution();
			~srv_funcobj_reset_particles_normal_distribution();
			int clear();
		public:
			bool is_called();
		public:
			int get_average(double *x, double *y, double *theta);
			int get_covariance( double *cov );
			int get_size(int *size);
		public:
			bool callback(request_t& request, response_t& response);
		};
		// <--- callback function object

	}
} // <--- message type definition


// ---> constant definition
namespace gnd {
	namespace particle_localizer {
		const particle_weight_t WEIGHT_T_EPSILON	= FLT_EPSILON;
	}
} // ---> constant definition


// ---> callback function definition
namespace gnd {
	namespace particle_localizer {
		srv_funcobj_reset_particles_normal_distribution::srv_funcobj_reset_particles_normal_distribution() {
			clear();
		}

		srv_funcobj_reset_particles_normal_distribution::~srv_funcobj_reset_particles_normal_distribution() {
		}

		int srv_funcobj_reset_particles_normal_distribution::clear() {
			is_called_ = false;
			return 0;
		}

		bool srv_funcobj_reset_particles_normal_distribution::is_called() {
			return is_called_;
		}


		int srv_funcobj_reset_particles_normal_distribution::get_average(double *x, double *y, double *theta) {
			*x = average_.x;
			*y = average_.y;
			*theta = average_.theta;
			return 0;
		}

		int srv_funcobj_reset_particles_normal_distribution::get_covariance( double *cov ) {
			memcpy( cov, covariance_, sizeof(covariance_) );
			return 0;
		}

		int srv_funcobj_reset_particles_normal_distribution::get_size(int *size) {
			*size = size_;
			return 0;
		}

		bool srv_funcobj_reset_particles_normal_distribution::callback(request_t& request, response_t& response) {
			if(is_called_) {
				response.ret = false;
			}
			else if( request.size < 0 ) {
				response.ret = false;
			}
			else {
				average_.x = request.x;
				average_.y = request.y;
				average_.theta = request.theta;
				memcpy(covariance_, request.covariance.data(), sizeof(covariance_));
				size_ = request.size;
				is_called_ = true;
				response.ret = true;
			}

			return true;
		}

	}
} // ---> callback function definition

#include "gnd/gnd_particle_localizer_config.hpp"



#endif // gnd_particle_localizer_HPP
